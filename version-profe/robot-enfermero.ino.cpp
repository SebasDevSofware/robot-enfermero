// ===================================================================
// | ROBOT ENFERMERO - RECEPTOR ESP-NOW + AP + IA + MOTORES          |
// | Modo: WIFI_AP_STA - Canal fijo 1 - Servidor Web integrado       |
// | Versión: FINAL - 100% COMUNICACIÓN EFECTIVA                     |
// ===================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include "esp_mac.h"

// ==================== CONFIGURACIÓN DE RED ====================
#define AP_SSID "BIOCHRONORITMIC_IA_CENTER"
#define AP_PASS "12345678"
#define ESP_NOW_CHANNEL 1

// ==================== CONFIGURACIÓN PINES =====================
#define BUZZER_PIN 18
#define LED_PIN 22
// Motor A (Izquierdo)
#define IN1_MOTORES 25
#define IN2_MOTORES 26
#define ENA_MOTORES 4
// Motor B (Derecho)
#define IN3_MOTORES 32
#define IN4_MOTORES 33
#define ENB_MOTORES 12
// Sensor Ultrasónico HC-SR04
#define TRIG_PIN 5 // Naranja
#define ECHO_PIN 35 // Verde

// ==================== UMBRALES MÉDICOS ====================
#define HR_NORMAL_MIN 60
#define HR_NORMAL_MAX 100
#define SPO2_NORMAL_MIN 96
#define SPO2_CRITICO 88
#define DISTANCIA_SEGURIDAD_CM 30
#define DISTANCIA_CRITICA_CM 10
#define TEMP_FIEBRE 38.0f
#define TEMP_GOLPE_CALOR 40.0f
#define TEMP_HIPOTERMIA 35.0f

// ==================== ESTRUCTURA DE TELEMETRÍA ====================
// IMPORTANTE: Debe ser IDÉNTICA a la de la pulsera
typedef struct __attribute__((packed)) {
  uint32_t timestamp;      // 4 bytes
  int32_t heartRate;       // 4 bytes
  int32_t spo2;            // 4 bytes
  float temperature;       // 4 bytes (temperatura ambiental)
  float pressure;          // 4 bytes
  float roll;              // 4 bytes
  float pitch;             // 4 bytes
  float yaw;               // 4 bytes
  float magnitude;         // 4 bytes
  uint8_t fallDetected;    // 1 byte
  uint8_t signalQuality;   // 1 byte
  uint8_t reserved[2];     // 2 bytes
} TelemetryPacket;         // TOTAL: 40 bytes

// ==================== ESTADOS PACIENTE ====================
typedef enum {
  ESTADO_NORMAL,
  ESTADO_MONITOREO,
  ESTADO_ALERTA,
  ESTADO_CRITICO,
  ESTADO_TAQUICARDIA,
  ESTADO_BRADICARDIA,
  ESTADO_HIPOXIA,
  ESTADO_GOLPE_CALOR,
  ESTADO_CAIDA,
  ESTADO_ACTIVIDAD_FISICA
} EstadoPaciente;

const char* ESTADO_STR[] = {
  "NORMAL", "MONITOREO", "ALERTA", "CRITICO",
  "TAQUICARDIA", "BRADICARDIA", "HIPOXIA",
  "GOLPE_CALOR", "CAIDA", "ACTIVIDAD_FISICA"
};

// ==================== VARIABLES GLOBALES ====================
TelemetryPacket processedData;
volatile bool mensaje_recibido = false;
bool fallUrgent = false;

uint8_t nivelRiesgoIA = 0;
EstadoPaciente estadoPaciente = ESTADO_NORMAL;
char diagnosticoIA[128] = "";
char recomendacionIA[128] = "";
bool killSwitchActive = false;

float distanciaObstaculo = 999.0f;
float altitudEstimada = 0.0f;
float temperaturaCorporalAprox = 36.5f;
char tipoActividad[20] = "";
char tendenciaHR[10] = "ESTABLE";
char tendenciaSpO2[10] = "ESTABLE";
float umbralSpO2Dinamico = 94.0f;

// Buffers para derivadas (ventana deslizante)
#define BUFFER_SIZE 20
float hrBuffer[BUFFER_SIZE];
float spo2Buffer[BUFFER_SIZE];
float tempBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;
bool bufferFull = false;

// Estadísticas comunicación
uint32_t packetsReceived = 0;
uint32_t packetsDropped = 0;
unsigned long lastPacketTime = 0;
unsigned long lastStatsTime = 0;

// Servidor Web
AsyncWebServer server(80);
AsyncEventSource events("/events");

// Queue para procesamiento RTOS
QueueHandle_t telemetryQueue = NULL;

// ==================== PROTOTIPOS ====================
void sistemaExpertoIA();
void actualizarUmbralesDinamicos();
void motoresInit();
void motoresDetener();
void motoresAvanzar(int velocidad);
void motoresRetroceder(int velocidad);
void motoresGirar(int velocidad);
float leerDistanciaUltrasonico();
float calcularAltitud(float presionHpa);
void alertaEmergencia(int duracionMs);
void TaskSafety(void *pvParameters);
void TaskActuators(void *pvParameters);
void TaskProcessTelemetry(void *pvParameters);

// ==================== CALLBACK ESP-NOW (RECEPCIÓN) ====================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(TelemetryPacket)) {
    TelemetryPacket *packet = (TelemetryPacket *)malloc(sizeof(TelemetryPacket));
    if (packet != NULL) {
      memcpy(packet, data, sizeof(TelemetryPacket));
      
      // Enviar ACK inmediato al emisor
      uint8_t ack = 0x01;
      esp_now_send(info->src_addr, &ack, 1);
      
      // Encolar para procesamiento fuera de la interrupción
      if (xQueueSend(telemetryQueue, &packet, 0) != pdTRUE) {
        free(packet);
        packetsDropped++;
      } else {
        packetsReceived++;
        lastPacketTime = millis();
      }
    }
  } else {
    packetsDropped++;
  }
}

// ==================== TAREA PROCESAMIENTO TELEMETRÍA ====================
void TaskProcessTelemetry(void *pvParameters) {
  TelemetryPacket *packet;
  
  while (true) {
    if (xQueueReceive(telemetryQueue, &packet, portMAX_DELAY) == pdTRUE) {
      if (packet != NULL) {
        memcpy(&processedData, packet, sizeof(TelemetryPacket));
        mensaje_recibido = true;
        fallUrgent = (packet->fallDetected == 1);
        free(packet);
      }
    }
  }
}

// ==================== TAREA SEGURIDAD (CORE 0 - Prioridad máxima) ====================
void TaskSafety(void *pvParameters) {
  while (true) {
    // Leer distancia cada 20ms (frecuencia ultrasónica)
    distanciaObstaculo = leerDistanciaUltrasonico();
    
    if (distanciaObstaculo < DISTANCIA_CRITICA_CM) {
      motoresDetener();
      killSwitchActive = true;
      
      // Retroceder si está demasiado cerca
      if (distanciaObstaculo < 5) {
        motoresRetroceder(150);
        vTaskDelay(pdMS_TO_TICKS(300));
        motoresDetener();
      }
    } else if (distanciaObstaculo < DISTANCIA_SEGURIDAD_CM) {
      killSwitchActive = true;
    } else {
      killSwitchActive = false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ==================== TAREA ACTUADORES (LED + BUZZER) ====================
void TaskActuators(void *pvParameters) {
  while (true) {
    if (nivelRiesgoIA >= 3 || fallUrgent) {
      // EMERGENCIA CRÍTICA: LED + BUZZER AGUDO + GIRO
      digitalWrite(LED_PIN, HIGH);
      tone(BUZZER_PIN, 2500, 200);
      
      if (!killSwitchActive) {
        motoresGirar(200); // Gira en círculos para llamar la atención
      } else {
        motoresDetener();  // Prioridad: no chocar si hay algo cerca
      }
    } else if (nivelRiesgoIA == 2) {
      // ALERTA: LED + BUZZER LENTO + MOTORES PARADOS
      digitalWrite(LED_PIN, HIGH);
      tone(BUZZER_PIN, 1000, 100);
      motoresDetener();
    } else {
      // NORMALIDAD
      digitalWrite(LED_PIN, LOW);
      noTone(BUZZER_PIN);
      motoresDetener();
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ==================== SENSOR ULTRASÓNICO ====================
float leerDistanciaUltrasonico() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999.0f;
  
  float distance = duration * 0.034 / 2;
  return (distance > 400) ? 400 : distance;
}

// ==================== CONTROL DE MOTORES ====================
void motoresInit() {
  pinMode(IN1_MOTORES, OUTPUT);
  pinMode(IN2_MOTORES, OUTPUT);
  pinMode(ENA_MOTORES, OUTPUT);
  pinMode(IN3_MOTORES, OUTPUT);
  pinMode(IN4_MOTORES, OUTPUT);
  pinMode(ENB_MOTORES, OUTPUT);
  motoresDetener();
}

void motoresDetener() {
  digitalWrite(IN1_MOTORES, LOW);
  digitalWrite(IN2_MOTORES, LOW);
  digitalWrite(IN3_MOTORES, LOW);
  digitalWrite(IN4_MOTORES, LOW);
  analogWrite(ENA_MOTORES, 0);
  analogWrite(ENB_MOTORES, 0);
}

void motoresAvanzar(int velocidad) {
  digitalWrite(IN1_MOTORES, HIGH);
  digitalWrite(IN2_MOTORES, LOW);
  digitalWrite(IN3_MOTORES, HIGH);
  digitalWrite(IN4_MOTORES, LOW);
  analogWrite(ENA_MOTORES, velocidad);
  analogWrite(ENB_MOTORES, velocidad);
}

void motoresRetroceder(int velocidad) {
  digitalWrite(IN1_MOTORES, LOW);
  digitalWrite(IN2_MOTORES, HIGH);
  digitalWrite(IN3_MOTORES, LOW);
  digitalWrite(IN4_MOTORES, HIGH);
  analogWrite(ENA_MOTORES, velocidad);
  analogWrite(ENB_MOTORES, velocidad);
}

void motoresGirar(int velocidad) {
  digitalWrite(IN1_MOTORES, HIGH); // Motor A adelante
  digitalWrite(IN2_MOTORES, LOW);
  digitalWrite(IN3_MOTORES, LOW);  // Motor B atrás
  digitalWrite(IN4_MOTORES, HIGH);
  analogWrite(ENA_MOTORES, velocidad);
  analogWrite(ENB_MOTORES, velocidad);
}

// ==================== CÁLCULO DE ALTITUD ====================
float calcularAltitud(float presionHpa) {
  return 44330.0f * (1.0f - pow(presionHpa / 1013.25f, 0.1903f));
}

// ==================== ACTUALIZAR UMBRALES DINÁMICOS ====================
void actualizarUmbralesDinamicos() {
  altitudEstimada = calcularAltitud(processedData.pressure);
  
  if (altitudEstimada > 2500) {
    umbralSpO2Dinamico = 90.0f;
  } else if (altitudEstimada > 1500) {
    umbralSpO2Dinamico = 92.0f;
  } else {
    umbralSpO2Dinamico = 94.0f;
  }
}

// ==================== SISTEMA EXPERTO IA ====================
void sistemaExpertoIA() {
  if (processedData.timestamp == 0) return;
  
  actualizarUmbralesDinamicos();
  
  // Actualizar buffers para derivadas
  if (processedData.heartRate > 0) hrBuffer[bufferIndex] = processedData.heartRate;
  if (processedData.spo2 > 0) spo2Buffer[bufferIndex] = processedData.spo2;
  tempBuffer[bufferIndex] = processedData.temperature;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  if (bufferIndex == 0) bufferFull = true;
  
  // Calcular tendencias si hay suficientes datos
  if (bufferFull) {
    float sumHR = 0, sumSpO2 = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      sumHR += hrBuffer[i];
      sumSpO2 += spo2Buffer[i];
    }
    float avgHR = sumHR / BUFFER_SIZE;
    float avgSpO2 = sumSpO2 / BUFFER_SIZE;
    
    float sumHRDiff = 0, sumSpO2Diff = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      sumHRDiff += (hrBuffer[i] - avgHR) * i;
      sumSpO2Diff += (spo2Buffer[i] - avgSpO2) * i;
    }
    float sumISq = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) sumISq += i * i;
    
    float derivadaHR = sumHRDiff / sumISq;
    float derivadaSpO2 = sumSpO2Diff / sumISq;
    
    if (derivadaHR > 2) strcpy(tendenciaHR, "▲ SUBIENDO");
    else if (derivadaHR < -2) strcpy(tendenciaHR, "▼ BAJANDO");
    else strcpy(tendenciaHR, "● ESTABLE");
    
    if (derivadaSpO2 < -0.8) strcpy(tendenciaSpO2, "▼ CAIDA");
    else if (derivadaSpO2 > 0.8) strcpy(tendenciaSpO2, "▲ SUBIENDO");
    else strcpy(tendenciaSpO2, "● ESTABLE");
  }
  
  // Clasificar actividad por magnitud
  if (processedData.magnitude > 3.5) strcpy(tipoActividad, "CORRIENDO");
  else if (processedData.magnitude > 2.0) strcpy(tipoActividad, "CAMINANDO");
  else if (abs(processedData.pitch) > 70) strcpy(tipoActividad, "ACOSTADO");
  else strcpy(tipoActividad, "REPOSO");
  
  // Cálculo de riesgos individuales
  float riesgoHR = 0, riesgoSpO2 = 0, riesgoTemp = 0, riesgoMov = 0;
  
  if (processedData.heartRate > 0) {
    if (processedData.heartRate > 120) {
      riesgoHR = min(1.0f, 0.7f + (processedData.heartRate - 120) / 50.0f);
      estadoPaciente = ESTADO_TAQUICARDIA;
    } else if (processedData.heartRate < 50) {
      riesgoHR = min(1.0f, 0.6f + (50 - processedData.heartRate) / 30.0f);
      estadoPaciente = ESTADO_BRADICARDIA;
    }
  }
  
  if (processedData.spo2 > 0) {
    if (processedData.spo2 <= umbralSpO2Dinamico - 10) {
      riesgoSpO2 = 1.0f;
      estadoPaciente = ESTADO_HIPOXIA;
    } else if (processedData.spo2 <= umbralSpO2Dinamico) {
      riesgoSpO2 = 0.7f;
    } else if (processedData.spo2 <= umbralSpO2Dinamico + 3) {
      riesgoSpO2 = 0.3f;
    }
  }
  
  if (processedData.temperature > 0) {
    if (processedData.temperature >= TEMP_GOLPE_CALOR) {
      riesgoTemp = 1.0f;
      estadoPaciente = ESTADO_GOLPE_CALOR;
    } else if (processedData.temperature >= TEMP_FIEBRE) {
      riesgoTemp = min(1.0f, 0.5f + (processedData.temperature - TEMP_FIEBRE) / 4.0f);
    } else if (processedData.temperature <= TEMP_HIPOTERMIA) {
      riesgoTemp = min(1.0f, 0.8f + (TEMP_HIPOTERMIA - processedData.temperature) / 5.0f);
    }
  }
  
  if (processedData.fallDetected) {
    riesgoMov = 1.0f;
    estadoPaciente = ESTADO_CAIDA;
  }
  
  temperaturaCorporalAprox = processedData.temperature > 0 ? processedData.temperature : 36.5f;
  
  // Riesgo total ponderado
  float riesgoTotal = (riesgoHR * 0.35f) + (riesgoSpO2 * 0.35f) + 
                      (riesgoTemp * 0.15f) + (riesgoMov * 0.15f);
  if (riesgoTotal > 1.0f) riesgoTotal = 1.0f;
  
  // Nivel de riesgo final
  if (riesgoTotal >= 0.7f || processedData.fallDetected) {
    nivelRiesgoIA = 3;
    estadoPaciente = ESTADO_CRITICO;
  } else if (riesgoTotal >= 0.4f) {
    nivelRiesgoIA = 2;
    estadoPaciente = ESTADO_ALERTA;
  } else if (riesgoTotal >= 0.15f) {
    nivelRiesgoIA = 1;
    estadoPaciente = ESTADO_MONITOREO;
  } else {
    nivelRiesgoIA = 0;
    estadoPaciente = ESTADO_NORMAL;
  }
  
  // Generar diagnóstico y recomendación
  memset(diagnosticoIA, 0, sizeof(diagnosticoIA));
  memset(recomendacionIA, 0, sizeof(recomendacionIA));
  
  if (nivelRiesgoIA >= 2) {
    if (riesgoHR > 0.5f) {
      if (processedData.heartRate < 50) {
        strcat(diagnosticoIA, "BRADICARDIA SEVERA. ");
      } else {
        strcat(diagnosticoIA, "TAQUICARDIA SEVERA. ");
      }
      strcat(recomendacionIA, "EVALUACIÓN CARDIOLÓGICA URGENTE. ");
    }
    if (riesgoSpO2 > 0.5f) {
      strcat(diagnosticoIA, "HIPOXEMIA CRÍTICA. ");
      strcat(recomendacionIA, "ADMINISTRAR OXÍGENO SUPLEMENTARIO. ");
    }
    if (riesgoTemp > 0.5f) {
      strcat(diagnosticoIA, "ALTERACIÓN TÉRMICA SEVERA. ");
      strcat(recomendacionIA, "CONTROL TÉRMICO INMEDIATO. ");
    }
    if (processedData.fallDetected) {
      strcat(diagnosticoIA, "CAÍDA DETECTADA. ");
      strcat(recomendacionIA, "VERIFICAR ESTADO DE CONCIENCIA. ");
    }
    alertaEmergencia(15);
  } else if (nivelRiesgoIA == 1) {
    if (riesgoHR > 0.2f) strcat(diagnosticoIA, "ALTERACIÓN CARDÍACA LEVE. ");
    if (riesgoSpO2 > 0.2f) strcat(diagnosticoIA, "DESATURACIÓN LEVE. ");
    strcat(recomendacionIA, "MONITOREO CONTINUO. ");
  } else {
    strcat(diagnosticoIA, "PARÁMETROS NORMALES.");
    strcat(recomendacionIA, "CONTINÚE MONITOREO.");
  }
}

// ==================== ALERTA DE EMERGENCIA ====================
void alertaEmergencia(int duracionMs) {
  for (int i = 0; i < duracionMs / 50; i++) {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 2000, 50);
    vTaskDelay(pdMS_TO_TICKS(25));
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

// ==================== HTML DASHBOARD ====================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>BIOCHRONORITMIC | IA MEDICAL ROBOT</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { background: #0a0f0f; color: #00ffcc; font-family: 'Courier New', monospace; padding: 15px; }
        .container { max-width: 800px; margin: 0 auto; }
        .header { border-bottom: 2px solid #00ffcc; padding-bottom: 10px; margin-bottom: 20px; display: flex; justify-content: space-between; flex-wrap: wrap; }
        .badge { display: inline-block; padding: 4px 12px; border-radius: 20px; font-size: 12px; }
        .badge-critical { background: #ff0000; color: white; animation: pulse 1s infinite; }
        .badge-warning { background: #ff6600; color: white; }
        .badge-normal { background: #00cc66; color: white; }
        .grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px; }
        .card { background: rgba(0, 30, 30, 0.7); border: 1px solid #00ffcc33; padding: 15px; border-radius: 8px; }
        .card-full { grid-column: span 2; }
        .card-title { font-size: 10px; text-transform: uppercase; opacity: 0.6; margin-bottom: 8px; letter-spacing: 1px; }
        .value { font-size: 32px; font-weight: bold; }
        .value-large { font-size: 24px; }
        .unit { font-size: 12px; opacity: 0.5; }
        .trend-up { color: #ff4444; }
        .trend-down { color: #ffaa44; }
        .trend-stable { color: #00ffcc; }
        @keyframes pulse { 0%,100% { opacity: 1; } 50% { opacity: 0.6; } }
        .fall-alert { animation: pulse 0.5s infinite; background: #ff000033; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <span style="font-weight: bold;">BIOCHRONORITMIC_IA_CENTER</span>
            <span id="status-badge" class="badge badge-normal">● CONECTADO</span>
        </div>
        
        <div class="grid">
            <div class="card">
                <div class="card-title">❤️ FRECUENCIA CARDIACA</div>
                <div><span id="hr" class="value">--</span><span class="unit"> BPM</span></div>
                <div><span id="hr-trend" class="trend-stable" style="font-size:11px">--</span></div>
            </div>
            
            <div class="card">
                <div class="card-title">🫁 SPO2</div>
                <div><span id="spo2" class="value">--</span><span class="unit"> %</span></div>
                <div><span id="spo2-trend" class="trend-stable" style="font-size:11px">--</span></div>
            </div>
            
            <div class="card">
                <div class="card-title">🌡️ TEMPERATURA</div>
                <div><span id="temp" class="value">--</span><span class="unit"> °C</span></div>
            </div>
            
            <div class="card">
                <div class="card-title">🏔️ ALTITUD</div>
                <div><span id="alt" class="value">--</span><span class="unit"> m</span></div>
                <div style="font-size:11px"><span id="pres">--</span> hPa</div>
            </div>
            
            <div class="card card-full" id="status-card">
                <div class="card-title">📊 ESTADO GLOBAL</div>
                <div><span id="estado" class="value value-large">--</span></div>
                <div id="diagnostico" style="font-size:12px; margin-top:12px; color:#ffffffcc">--</div>
                <div id="recomendacion" style="font-size:11px; margin-top:8px; opacity:0.7">--</div>
            </div>
            
            <div class="card card-full">
                <div class="card-title">🏃 MOVIMIENTO</div>
                <div style="display:flex; justify-content:space-between; flex-wrap:wrap;">
                    <span>ROLL: <span id="roll">--</span>°</span>
                    <span>PITCH: <span id="pitch">--</span>°</span>
                    <span>YAW: <span id="yaw">--</span>°</span>
                    <span>MAG: <span id="mag">--</span>G</span>
                </div>
                <div style="margin-top:10px">ACTIVIDAD: <strong id="actividad">--</strong></div>
            </div>
            
            <div class="card card-full">
                <div class="card-title">🤖 ROBOT</div>
                <div style="display:flex; justify-content:space-between; flex-wrap:wrap;">
                    <span>OBSTÁCULO: <span id="distancia">--</span> cm</span>
                    <span>KILL SWITCH: <span id="kill">INACTIVO</span></span>
                    <span>CAÍDA: <span id="fall">NO</span></span>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        const source = new EventSource('/events');
        source.addEventListener('ia', function(e) {
            const d = JSON.parse(e.data);
            
            document.getElementById('hr').innerHTML = d.heartRate > 0 ? d.heartRate : '--';
            document.getElementById('spo2').innerHTML = d.spo2 > 0 ? d.spo2 : '--';
            document.getElementById('temp').innerHTML = d.temperaturaCorporalAprox.toFixed(1);
            document.getElementById('alt').innerHTML = d.altitudEstimada.toFixed(0);
            document.getElementById('pres').innerHTML = d.pressure.toFixed(1);
            document.getElementById('estado').innerHTML = d.estadoGlobal;
            document.getElementById('diagnostico').innerHTML = d.diagnosticoIA;
            document.getElementById('recomendacion').innerHTML = d.recomendacionIA;
            document.getElementById('roll').innerHTML = d.roll.toFixed(0);
            document.getElementById('pitch').innerHTML = d.pitch.toFixed(0);
            document.getElementById('yaw').innerHTML = d.yaw.toFixed(0);
            document.getElementById('mag').innerHTML = d.magnitude.toFixed(2);
            document.getElementById('actividad').innerHTML = d.tipoActividad;
            document.getElementById('distancia').innerHTML = d.distanciaObstaculo.toFixed(0);
            document.getElementById('fall').innerHTML = d.fallDetected ? '⚠ SI ⚠' : 'NO';
            document.getElementById('kill').innerHTML = d.killSwitchActive ? 'ACTIVO' : 'INACTIVO';
            document.getElementById('hr-trend').innerHTML = d.tendenciaHR;
            document.getElementById('spo2-trend').innerHTML = d.tendenciaSpO2;
            
            const statusCard = document.getElementById('status-card');
            const statusBadge = document.getElementById('status-badge');
            
            if(d.nivelRiesgoIA >= 3) {
                statusCard.classList.add('fall-alert');
                statusBadge.className = 'badge badge-critical';
                statusBadge.innerHTML = '🔴 CRÍTICO 🔴';
            } else if(d.nivelRiesgoIA >= 2) {
                statusCard.classList.remove('fall-alert');
                statusBadge.className = 'badge badge-warning';
                statusBadge.innerHTML = '⚠ ALERTA ⚠';
            } else {
                statusCard.classList.remove('fall-alert');
                statusBadge.className = 'badge badge-normal';
                statusBadge.innerHTML = '● CONECTADO';
            }
        });
        
        source.onerror = function(e) { console.error("SSE Error", e); };
    </script>
</body>
</html>
)rawliteral";

// ==================== IMPRIMIR MAC DEL ROBOT ====================
void printRobotMAC() {
  uint8_t mac[6];
  // En el core 3.x, esp_read_mac requiere la librería esp_mac.h
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
    Serial.print("[MAC] Mi dirección MAC (copia esto en la pulsera): ");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else {
    Serial.println("[ERROR] No se pudo leer la dirección MAC");
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║ ROBOT ENFERMERO BIOCHRONORITMIC      ║");
  Serial.println("║ Modo: AP + ESP-NOW Receptor          ║");
  Serial.println("╚════════════════════════════════════════╝");
  
  // Inicializar pines
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  motoresInit();
  
  // Tono de inicio
  digitalWrite(BUZZER_PIN, HIGH);
  delay(250);
  digitalWrite(BUZZER_PIN, LOW);
  
  // ==================== CONFIGURACIÓN WIFI (AP + ESP-NOW) ====================
  WiFi.mode(WIFI_AP_STA);
  
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(AP_SSID, AP_PASS, ESP_NOW_CHANNEL);
  
  Serial.print("[WIFI] AP '");
  Serial.print(AP_SSID);
  Serial.print("' creado en canal ");
  Serial.println(ESP_NOW_CHANNEL);
  Serial.print("[WIFI] IP: ");
  Serial.println(WiFi.softAPIP());
  
  printRobotMAC();
  
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  
  // Configurar potencia máxima
  esp_wifi_set_max_tx_power(78);
  esp_wifi_set_ps(WIFI_PS_NONE);
  
  // ==================== CONFIGURACIÓN ESP-NOW ====================
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init falló");
    while (1) delay(100);
  }
  
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
  
  // Añadir broadcast peer para enviar ACKs
  esp_now_peer_info_t broadcastPeer = {};
  for (int i = 0; i < 6; i++) broadcastPeer.peer_addr[i] = 0xFF;
  broadcastPeer.channel = ESP_NOW_CHANNEL;
  broadcastPeer.encrypt = false;
  broadcastPeer.ifidx = WIFI_IF_AP;
  esp_now_add_peer(&broadcastPeer);
  
  Serial.print("[ESP-NOW] Receptor listo - Canal ");
  Serial.println(ESP_NOW_CHANNEL);
  
  // ==================== SERVIDOR WEB ====================
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });
  
  server.addHandler(&events);
  server.begin();
  Serial.println("[WEB] Servidor HTTP iniciado en http://192.168.4.1");
  
  // ==================== COLAS Y TAREAS RTOS ====================
  telemetryQueue = xQueueCreate(20, sizeof(TelemetryPacket*));
  
  // Crear tareas en cores específicos
  xTaskCreatePinnedToCore(TaskProcessTelemetry, "ProcessTelemetry", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskSafety, "SafetyTask", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(TaskActuators, "ActuatorsTask", 4096, NULL, 1, NULL, 1);
  
  // Inicializar buffers
  for (int i = 0; i < BUFFER_SIZE; i++) {
    hrBuffer[i] = 75;
    spo2Buffer[i] = 97;
    tempBuffer[i] = 36.5f;
  }
  
  Serial.println("\n[ROBOT] ✅ Sistema listo - Esperando datos de la pulsera...");
  Serial.println("[INFO] Conéctate a la WiFi 'BIOCHRONORITMIC_IA_CENTER' y ve a http://192.168.4.1\n");
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
}

// ==================== LOOP PRINCIPAL ====================
void loop() {
  static unsigned long lastWebSend = 0;
  static unsigned long lastConsoleLog = 0;
  
  if (mensaje_recibido) {
    sistemaExpertoIA();
    
    // Enviar al navegador cada 200ms (no saturar el AP)
    unsigned long now = millis();
    if (now - lastWebSend > 200) {
      StaticJsonDocument<1024> doc;
      doc["heartRate"] = processedData.heartRate;
      doc["spo2"] = processedData.spo2;
      doc["temperature"] = processedData.temperature;
      doc["pressure"] = processedData.pressure;
      doc["roll"] = processedData.roll;
      doc["pitch"] = processedData.pitch;
      doc["yaw"] = processedData.yaw;
      doc["magnitude"] = processedData.magnitude;
      doc["estadoGlobal"] = ESTADO_STR[estadoPaciente];
      doc["fallDetected"] = processedData.fallDetected;
      doc["signalQuality"] = processedData.signalQuality;
      doc["nivelRiesgoIA"] = nivelRiesgoIA;
      doc["diagnosticoIA"] = diagnosticoIA;
      doc["recomendacionIA"] = recomendacionIA;
      doc["temperaturaCorporalAprox"] = temperaturaCorporalAprox;
      doc["tipoActividad"] = tipoActividad;
      doc["altitudEstimada"] = altitudEstimada;
      doc["distanciaObstaculo"] = distanciaObstaculo;
      doc["killSwitchActive"] = killSwitchActive ? 1 : 0;
      doc["tendenciaHR"] = tendenciaHR;
      doc["tendenciaSpO2"] = tendenciaSpO2;
      
      String jsonResponse;
      serializeJson(doc, jsonResponse);
      events.send(jsonResponse.c_str(), "ia", millis());
      lastWebSend = now;
    }
    
    // Log en consola cada 5 segundos
    if (now - lastConsoleLog > 5000) {
      Serial.println("\n╔════════════════════════════════════════════════╗");
      Serial.printf("║ ESTADO: %-37s║\n", ESTADO_STR[estadoPaciente]);
      Serial.printf("║ RIESGO: NIVEL %d                                 ║\n", nivelRiesgoIA);
      Serial.println("╠════════════════════════════════════════════════╣");
      Serial.printf("║ ❤️ BPM: %4d  |  🫁 SpO2: %3d%%  |  🌡️ Temp: %4.1f°C  ║\n",
                    processedData.heartRate, processedData.spo2, temperaturaCorporalAprox);
      Serial.printf("║ 📊 HR: %-10s | SpO2: %-10s              ║\n", tendenciaHR, tendenciaSpO2);
      Serial.printf("║ 🏃 Actividad: %-30s║\n", tipoActividad);
      Serial.printf("║ 🏔️ Altitud: %5.0fm | SpO2 Umbral: %4.0f%%            ║\n",
                    altitudEstimada, umbralSpO2Dinamico);
      Serial.println("╠════════════════════════════════════════════════╣");
      Serial.printf("║ 💊 DIAG: %-33s║\n", diagnosticoIA);
      Serial.printf("║ 💊 RECO: %-33s║\n", recomendacionIA);
      Serial.println("╠════════════════════════════════════════════════╣");
      Serial.printf("║ 📡 Paquetes: Recv:%lu | Drop:%lu | Tasa:%.1f%%      ║\n",
                    packetsReceived, packetsDropped,
                    packetsReceived + packetsDropped > 0 ? 
                    100.0f * packetsReceived / (packetsReceived + packetsDropped) : 0);
      Serial.printf("║ 📍 Obstáculo: %5.1fcm | 🔪 Kill: %-8s            ║\n",
                    distanciaObstaculo, killSwitchActive ? "ACTIVO" : "INACTIVO");
      Serial.println("╚════════════════════════════════════════════════╝");
      lastConsoleLog = now;
    }
    
    mensaje_recibido = false;
  }
  
  // Detectar pérdida de comunicación
  static unsigned long lastCommsAlert = 0;
  unsigned long now = millis();
  if (lastPacketTime > 0 && (now - lastPacketTime) > 5000 && (now - lastCommsAlert) > 10000) {
    Serial.println("[ALERTA] ⚠️ ¡Pérdida de comunicación con la pulsera! ⚠️");
    lastCommsAlert = now;
  }
  
  vTaskDelay(pdMS_TO_TICKS(10));
}