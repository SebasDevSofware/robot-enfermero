#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <UrlEncode.h>

// CONEXIONES

// ==================== LÓGICA DIFUSA (FUZZY LOGIC ENGINE) ====================
float trapmf(float x, float a, float b, float c, float d) {
    if (x <= a || x >= d) return 0.0f;
    if (x >= b && x <= c) return 1.0f;
    if (x > a && x < b) return (x - a) / (b - a);
    if (x > c && x < d) return (d - x) / (d - c);
    return 0.0f;
}

float trimf(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0f;
    if (x == b) return 1.0f;
    if (x > a && x < b) return (x - a) / (b - a);
    if (x > b && x < c) return (c - x) / (c - b);
    return 0.0f;
}

// ==================== COMUNICACIÓN INTER-CORE ====================
SemaphoreHandle_t dataMutex;
QueueHandle_t motorQueue;

struct MotorCommand {
    int leftSpeed;
    int rightSpeed;
    bool brake;
};

// Handles de Tareas
TaskHandle_t TaskHardwareHandle;
TaskHandle_t TaskProtocolHandle;

// --- CONFIGURACIÓN WHATSAPP (CallMeBot) ---
#define WIFI_SSID "TU_SSID"
#define WIFI_PASS "TU_PASSWORD"
#define CALLMEBOT_PHONE "+584125374909"
#define CALLMEBOT_API_KEY "1404207"

struct WhatsAppAlert {
  int riskLevel;
  char diagnosis[128];
  char recommendation[128];
  int hr;
  int spo2;
  float temp;
  bool fall;
  char state[20];
};

QueueHandle_t whatsappQueue;
unsigned long lastWhatsAppTime = 0;
const unsigned long WHATSAPP_COOLDOWN = 60000;

// --- CONFIGURACIÓN SERVIDOR WEB Y SSE ---
AsyncWebServer server(80);
AsyncEventSource events("/events");

// Dashboard HTML/CSS/JS Professional Elite
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ALMA ELITE | MEDICAL CORE</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        :root {
            --neon-cyan: #00f2ff;
            --neon-red: #ff003c;
            --neon-orange: #ff9d00;
            --bg-black: #050505;
            --glass: rgba(255, 255, 255, 0.03);
            --border: rgba(0, 242, 255, 0.2);
        }
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            background: var(--bg-black);
            color: var(--neon-cyan);
            font-family: 'Segoe UI', Roboto, 'Courier New', monospace;
            overflow-x: hidden;
            background-image: radial-gradient(circle at 50% 50%, #101010 0%, #050505 100%);
            min-height: 100vh;
        }
        .app-container { padding: 20px; max-width: 1400px; margin: 0 auto; }
        .header {
            display: flex; justify-content: space-between; align-items: center;
            padding: 15px; border-bottom: 1px solid var(--border);
            margin-bottom: 30px; backdrop-filter: blur(10px);
            position: sticky; top: 0; z-index: 100;
        }
        .header h1 { font-size: 1.2rem; letter-spacing: 5px; text-transform: uppercase; font-weight: 300; }
        
        .main-grid {
            display: grid;
            grid-template-columns: repeat(12, 1fr);
            gap: 20px;
        }

        .card {
            background: var(--glass);
            border: 1px solid var(--border);
            border-radius: 4px;
            padding: 20px;
            position: relative;
            backdrop-filter: blur(5px);
            transition: all 0.3s ease;
        }
        .card:hover { border-color: var(--neon-cyan); box-shadow: 0 0 15px rgba(0, 242, 255, 0.1); }
        
        .status-hero { grid-column: span 12; text-align: center; border-width: 2px; }
        .risk-0 { border-color: var(--neon-cyan); color: var(--neon-cyan); }
        .risk-1 { border-color: var(--neon-orange); color: var(--neon-orange); }
        .risk-2 { border-color: var(--neon-orange); animation: pulse 2s infinite; }
        .risk-3 { border-color: var(--neon-red); color: var(--neon-red); animation: blink 0.5s infinite; }

        @keyframes pulse { 0% { box-shadow: 0 0 0 0 rgba(255, 157, 0, 0.4); } 70% { box-shadow: 0 0 0 15px rgba(255, 157, 0, 0); } 100% { box-shadow: 0 0 0 0 rgba(255, 157, 0, 0); } }
        @keyframes blink { 0% { opacity: 1; border-color: var(--neon-red); } 50% { opacity: 0.7; border-color: transparent; } 100% { opacity: 1; border-color: var(--neon-red); } }

        .val-display { display: flex; align-items: baseline; gap: 10px; margin: 10px 0; }
        .val-num { font-size: 3.5rem; font-weight: 200; }
        .val-unit { font-size: 0.9rem; opacity: 0.6; }

        .vitals-grid { grid-column: span 8; display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .enviro-grid { grid-column: span 4; display: flex; flex-direction: column; gap: 20px; }

        @media (max-width: 900px) {
            .vitals-grid, .enviro-grid { grid-column: span 12; }
            .vitals-grid { grid-template-columns: 1fr; }
        }

        .chart-container { height: 120px; width: 100%; margin-top: 15px; }
        .label { font-size: 0.7rem; text-transform: uppercase; letter-spacing: 2px; opacity: 0.7; border-left: 2px solid; padding-left: 8px; }
        
        .indicator { font-size: 0.8rem; margin-top: 10px; padding: 5px 10px; background: rgba(0,0,0,0.3); border-radius: 2px; display: inline-block; }
    </style>
</head>
<body>
    <div class="app-container">
        <header class="header">
            <h1>ALMA_ELITE_CORE_v2.0</h1>
            <div id="connection-status" style="font-size: 0.7rem;">● SYSTEM_ONLINE</div>
        </header>

        <div class="main-grid">
            <div id="risk-card" class="card status-hero risk-0">
                <div class="label">IA_DIAGNOSTIC_ENGINE</div>
                <h2 id="diag" style="margin: 15px 0; font-weight: 300; font-size: 2.2rem;">CALIBRANDO...</h2>
                <p id="reco" style="opacity: 0.8; font-size: 0.9rem;">Sincronizando telemetría...</p>
                <div class="indicator" id="risk-lv">NIVEL_RIESGO: -</div>
            </div>

            <div class="vitals-grid">
                <div class="card">
                    <div class="label" style="border-color: var(--neon-cyan);">Heart Rate</div>
                    <div class="val-display">
                        <span id="hr" class="val-num">--</span>
                        <span class="val-unit">BPM</span>
                    </div>
                    <div class="indicator" id="hr-trend">TENDENCIA: --</div>
                    <div class="chart-container"><canvas id="hrChart"></canvas></div>
                </div>

                <div class="card">
                    <div class="label" style="border-color: var(--neon-red);">SpO2 Level</div>
                    <div class="val-display">
                        <span id="spo2" class="val-num">--</span>
                        <span class="val-unit">%</span>
                    </div>
                    <div class="indicator" id="spo2-trend">TENDENCIA: --</div>
                    <div class="chart-container"><canvas id="spo2Chart"></canvas></div>
                </div>

                <div class="card">
                    <div class="label">Body Temp (IA Est.)</div>
                    <div class="val-display">
                        <span id="temp-ia" class="val-num">--.-</span>
                        <span class="val-unit">°C</span>
                    </div>
                    <div class="indicator">AMB: <span id="temp-amb">--</span>°C</div>
                </div>

                <div class="card">
                    <div class="label">IMU Matrix & Activity</div>
                    <div style="margin-top: 15px;">
                        <p style="font-size: 0.8rem; margin-bottom: 5px;">ACT: <span id="activity" style="color:white">--</span></p>
                        <p style="font-size: 0.7rem; opacity: 0.6;">R: <span id="roll">0</span>° | P: <span id="pitch">0</span>° | Y: <span id="yaw">0</span>°</p>
                        <p style="font-size: 0.7rem; opacity: 0.6; margin-top:5px;">MAG: <span id="mag">0.0</span>G</p>
                    </div>
                    <div id="fall-alert" style="display:none; color: var(--neon-red); font-weight: bold; margin-top:10px;">!!! FALL_DETECTED !!!</div>
                </div>
            </div>

            <div class="enviro-grid">
                <div class="card">
                    <div class="label">Atmospheric Control</div>
                    <div style="margin-top: 10px; font-size: 0.8rem;">
                        <p>PRESIÓN: <span id="pres">0</span> hPa</p>
                        <p>ALTITUD: <span id="altitude">0</span> m</p>
                    </div>
                </div>
                <div class="card">
                    <div class="label">Obstacle & Safety</div>
                    <div style="margin-top: 10px; font-size: 0.8rem;">
                        <p>DIST: <span id="distance">--</span> cm</p>
                        <p>KILL_SWITCH: <span id="kill-switch" style="color:var(--neon-red)">INACTIVO</span></p>
                    </div>
                </div>
                <div class="card">
                    <div class="label">Signal Integrity</div>
                    <div style="margin-top: 10px; font-size: 0.8rem;">
                        <p>QUALITY: <span id="sig">0</span>%</p>
                        <p>UPTIME: <span id="uptime">0</span>s</p>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
        const ctxHr = document.getElementById('hrChart').getContext('2d');
        const ctxSp = document.getElementById('spo2Chart').getContext('2d');
        
        const chartOptions = {
            responsive: true, maintainAspectRatio: false,
            scales: { x: { display: false }, y: { display: false } },
            plugins: { legend: { display: false } },
            elements: { line: { tension: 0.4 }, point: { radius: 0 } }
        };

        const hrChart = new Chart(ctxHr, {
            type: 'line',
            data: { labels: Array(30).fill(''), datasets: [{ data: [], borderColor: '#00f2ff', borderWidth: 2 }] },
            options: chartOptions
        });

        const spo2Chart = new Chart(ctxSp, {
            type: 'line',
            data: { labels: Array(30).fill(''), datasets: [{ data: [], borderColor: '#ff003c', borderWidth: 2 }] },
            options: chartOptions
        });

        const source = new EventSource('/events');
        source.onmessage = function(e) {
            const d = JSON.parse(e.data);
            
            document.getElementById('hr').innerText = d.heartRate;
            document.getElementById('spo2').innerText = d.spo2;
            document.getElementById('temp-ia').innerText = d.temperatutraAproxIA.toFixed(1);
            document.getElementById('temp-amb').innerText = d.temperature.toFixed(1);
            document.getElementById('diag').innerText = d.diagnosticoIA;
            document.getElementById('reco').innerText = d.recomendacionIA;
            document.getElementById('activity').innerText = d.tipoActividad;
            document.getElementById('risk-lv').innerText = "NIVEL_RIESGO: " + ["ESTABLE", "OBSERVACIÓN", "ADVERTENCIA", "CRÍTICO"][d.nivelRiesgoIA];
            document.getElementById('risk-card').className = "card status-hero risk-" + d.nivelRiesgoIA;
            
            document.getElementById('roll').innerText = d.roll.toFixed(0);
            document.getElementById('pitch').innerText = d.pitch.toFixed(0);
            document.getElementById('yaw').innerText = d.yaw.toFixed(0);
            document.getElementById('mag').innerText = d.magnitude.toFixed(2);
            document.getElementById('pres').innerText = d.pressure.toFixed(0);
            document.getElementById('altitude').innerText = d.altitudEstimada.toFixed(0);
            document.getElementById('distance').innerText = d.distanciaObstaculo > 0 ? d.distanciaObstaculo.toFixed(0) : "--";
            document.getElementById('kill-switch').innerText = d.killSwitchActive ? "ACTIVO" : "INACTIVO";
            document.getElementById('sig').innerText = d.signalQuality;
            document.getElementById('hr-trend').innerText = "TENDENCIA: " + d.tendenciaHR;
            document.getElementById('spo2-trend').innerText = "TENDENCIA: " + d.tendenciaSpO2;
            document.getElementById('fall-alert').style.display = d.fallDetected ? "block" : "none";

            // Update Charts
            hrChart.data.datasets[0].data.push(d.heartRate);
            if(hrChart.data.datasets[0].data.length > 30) hrChart.data.datasets[0].data.shift();
            hrChart.update('none');

            spo2Chart.data.datasets[0].data.push(d.spo2);
            if(spo2Chart.data.datasets[0].data.length > 30) spo2Chart.data.datasets[0].data.shift();
            spo2Chart.update('none');
        };
    </script>
</body>
</html>
)rawliteral";

// ==================== CONFIGURACIÓN PINES ====================

#define BUZZER_PIN 18
#define LED_PIN 23

// Motor A (Izquierdo)
#define IN1_MOTORES 25   // IN_A (Dirección)
#define IN2_MOTORES 26   // IN_B (Dirección)
#define ENA_MOTORES 4    // PWM_A (Velocidad)

// Motor B (Derecho)
#define IN3_MOTORES 32   // IN_A (Dirección)
#define IN4_MOTORES 33   // IN_B (Dirección)
#define ENB_MOTORES 2    // PWM_B (Velocidad)

// Sensor Ultrasónico HC-SR04
#define TRIG_PIN 17
#define ECHO_PIN 16

// ==================== MÁQUINA DE ESTADOS DEL PACIENTE ====================
enum EstadoPaciente {
  ESTADO_NORMAL = 0,
  ESTADO_ACTIVIDAD_FISICA,
  ESTADO_MONITOREO,
  ESTADO_ALERTA,
  ESTADO_CRITICO,
  ESTADO_CAIDA,
  ESTADO_TAQUICARDIA_REPOSO,
  ESTADO_BRADICARDIA,
  ESTADO_HIPOXIA,
  ESTADO_GOLPE_CALOR
};

const char* ESTADO_STR[] = {
  "NORMAL",
  "ACTIVIDAD_FISICA",
  "MONITOREO",
  "ALERTA",
  "CRITICO",
  "CAIDA",
  "TAQUICARDIA_REPOSO",
  "BRADICARDIA",
  "HIPOXIA",
  "GOLPE_CALOR"
};

// ==================== UMBRALES MÉDICOS DINÁMICOS ====================
#define HR_NORMAL_MIN 60
#define HR_NORMAL_MAX 100
#define HR_EJERCICIO_MAX 150

#define SPO2_NORMAL_MIN 95
#define SPO2_ALERTA 92
#define SPO2_CRITICO 88

#define TEMP_FIEBRE 38.0
#define TEMP_HIPOTERMIA 35.0
#define TEMP_GOLPE_CALOR 40.0

#define PRESION_NIVEL_MAR 1013.25
#define PRESION_ALTURA_MEDIA 800.0

#define UMBRAL_MAGNITUD_REPOSO 0.5f
#define UMBRAL_MAGNITUD_ACTIVIDAD 2.0f
#define UMBRAL_MAGNITUD_ALTA 3.5f

#define DISTANCIA_SEGURIDAD_CM 30
#define DISTANCIA_CRITICA_CM 10

// ==================== BÚFERES CIRCULARES PARA ANÁLISIS TEMPORAL ====================
#define BUFFER_SIZE 10

typedef struct {
  float data[BUFFER_SIZE];
  uint8_t head;
  uint8_t count;
} CircularBuffer;

CircularBuffer bufferHR;
CircularBuffer bufferSpO2;
CircularBuffer bufferTemp;

// Derivadas (velocidad de cambio)
float derivadaHR = 0.0f;
float derivadaSpO2 = 0.0f;
float derivadaTemp = 0.0f;

// ==================== ESTRUCTURA DE TELEMETRÍA ====================
typedef struct __attribute__((packed)) {
  uint32_t timestamp;
  int32_t heartRate;
  int32_t spo2;
  float temperature;
  float pressure;
  float roll;
  float pitch;
  float yaw;
  float magnitude;
  char estadoGlobal[20];
  uint8_t fallDetected;
  uint8_t signalQuality;
} TelemetryPacket;

// ==================== CONFIGURACIÓN DE BATCH (LOTE) IA ====================
#define IA_BATCH_SIZE 20
TelemetryPacket batchBuffer[IA_BATCH_SIZE];
TelemetryPacket processedData;
uint8_t batchIndex = 0;
volatile bool mensaje_recibido = false;
bool fallUrgent = false;

// ==================== VARIABLES GLOBALES IA ELITE ====================
uint8_t nivelRiesgoIA = 0;
EstadoPaciente estadoPaciente = ESTADO_NORMAL;
char diagnosticoIA[128] = "";
char recomendacionIA[128] = "";
float temperatutraAproxIA = 36.5;
char tipoActividad[32] = "INDEFINIDA";
float altitudEstimada = 0.0f;
float umbralSpO2Dinamico = SPO2_ALERTA;
bool killSwitchActive = false;
float distanciaObstaculo = 0.0f;
char tendenciaHR[16] = "ESTABLE";
char tendenciaSpO2[16] = "ESTABLE";

// ==================== CONTROL DE ALARMA ASÍNCRONA ====================
unsigned long tiempoInicioAlerta = 0;
unsigned long duracionAlertaMs = 0;
bool alertaActiva = false;
int patronAlertaActual = 0;

// Patrones de alerta codificados
enum PatronAlerta {
  PATRON_NORMAL = 0,
  PATRON_MONITOREO,
  PATRON_ALERTA,
  PATRON_CRITICO,
  PATRON_CAIDA,
  PATRON_TAQUICARDIA,
  PATRON_BRADICARDIA,
  PATRON_HIPOXIA,
  PATRON_GOLPE_CALOR
};

// ==================== FUNCIONES DE BÚFER CIRCULAR ====================
void bufferInit(CircularBuffer* buf) {
  buf->head = 0;
  buf->count = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) buf->data[i] = 0;
}

void bufferPush(CircularBuffer* buf, float value) {
  buf->data[buf->head] = value;
  buf->head = (buf->head + 1) % BUFFER_SIZE;
  if (buf->count < BUFFER_SIZE) buf->count++;
}

float bufferGetDerivada(CircularBuffer* buf) {
  if (buf->count < 2) return 0.0f;
  uint8_t prev = (buf->head + BUFFER_SIZE - 2) % BUFFER_SIZE;
  uint8_t curr = (buf->head + BUFFER_SIZE - 1) % BUFFER_SIZE;
  return buf->data[curr] - buf->data[prev];
}

float bufferGetTendencia(CircularBuffer* buf) {
  if (buf->count < 3) return 0.0f;
  float sum = 0;
  int n = buf->count;
  for (int i = 0; i < n - 1; i++) {
    uint8_t idx1 = (buf->head + BUFFER_SIZE - n + i) % BUFFER_SIZE;
    uint8_t idx2 = (buf->head + BUFFER_SIZE - n + i + 1) % BUFFER_SIZE;
    sum += buf->data[idx2] - buf->data[idx1];
  }
  return sum / (n - 1);
}

// ==================== SENSOR ULTRASÓNICO (HC-SR04) ====================
float leerDistanciaUltrasonico() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long timeout = micros() + 30000; // 30ms timeout
  while (digitalRead(ECHO_PIN) == LOW && micros() < timeout);
  unsigned long startTime = micros();
  while (digitalRead(ECHO_PIN) == HIGH && micros() < timeout);
  unsigned long endTime = micros();

  unsigned long duration = endTime - startTime;
  float distance = duration * 0.034 / 2.0;
  return (distance > 0 && distance < 400) ? distance : -1;
}

bool evaluarKillSwitch() {
  distanciaObstaculo = leerDistanciaUltrasonico();
  if (distanciaObstaculo > 0 && distanciaObstaculo < DISTANCIA_SEGURIDAD_CM) {
    killSwitchActive = true;
    return true;
  }
  killSwitchActive = false;
  return false;
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
  digitalWrite(ENA_MOTORES, LOW);
  digitalWrite(ENB_MOTORES, LOW);
}

void motoresFrenadoEmergencia() {
  // Freno activo (ambos pines HIGH en BTS7960)
  digitalWrite(IN1_MOTORES, HIGH);
  digitalWrite(IN2_MOTORES, HIGH);
  digitalWrite(IN3_MOTORES, HIGH);
  digitalWrite(IN4_MOTORES, HIGH);
  digitalWrite(ENA_MOTORES, HIGH);
  digitalWrite(ENB_MOTORES, HIGH);
}

void motoresAvanzar(int velocidad) {
  if (killSwitchActive || evaluarKillSwitch()) {
    motoresFrenadoEmergencia();
    return;
  }
  digitalWrite(IN1_MOTORES, HIGH);
  digitalWrite(IN2_MOTORES, LOW);
  digitalWrite(IN3_MOTORES, HIGH);
  digitalWrite(IN4_MOTORES, LOW);
  analogWrite(ENA_MOTORES, velocidad);
  analogWrite(ENB_MOTORES, velocidad);
}

void motoresRetroceder(int velocidad) {
  if (killSwitchActive || evaluarKillSwitch()) {
    motoresFrenadoEmergencia();
    return;
  }
  digitalWrite(IN1_MOTORES, LOW);
  digitalWrite(IN2_MOTORES, HIGH);
  digitalWrite(IN3_MOTORES, LOW);
  digitalWrite(IN4_MOTORES, HIGH);
  analogWrite(ENA_MOTORES, velocidad);
  analogWrite(ENB_MOTORES, velocidad);
}

void motoresGirarIzquierda(int velocidad) {
  if (killSwitchActive || evaluarKillSwitch()) {
    motoresFrenadoEmergencia();
    return;
  }
  digitalWrite(IN1_MOTORES, LOW);
  digitalWrite(IN2_MOTORES, HIGH);
  digitalWrite(IN3_MOTORES, HIGH);
  digitalWrite(IN4_MOTORES, LOW);
  analogWrite(ENA_MOTORES, velocidad);
  analogWrite(ENB_MOTORES, velocidad);
}

void motoresGirarDerecha(int velocidad) {
  if (killSwitchActive || evaluarKillSwitch()) {
    motoresFrenadoEmergencia();
    return;
  }
  digitalWrite(IN1_MOTORES, HIGH);
  digitalWrite(IN2_MOTORES, LOW);
  digitalWrite(IN3_MOTORES, LOW);
  digitalWrite(IN4_MOTORES, HIGH);
  analogWrite(ENA_MOTORES, velocidad);
  analogWrite(ENB_MOTORES, velocidad);
}

// ==================== TAREAS ESPECÍFICAS DE NÚCLEO ====================

void TaskHardware(void *pvParameters) {
  Serial.println("[SYSTEM] TaskHardware iniciada en Core 1");
  MotorCommand cmd;
  unsigned long lastUltraRead = 0;

  for (;;) {
    // 1. Control de Motores (Consumir cola)
    if (xQueueReceive(motorQueue, &cmd, 10) == pdPASS) {
      if (cmd.brake) {
        motoresFrenadoEmergencia();
      } else {
        // Ejecutar movimiento
        if (cmd.leftSpeed > 0) {
          digitalWrite(IN1_MOTORES, HIGH); digitalWrite(IN2_MOTORES, LOW);
        } else if (cmd.leftSpeed < 0) {
          digitalWrite(IN1_MOTORES, LOW); digitalWrite(IN2_MOTORES, HIGH);
        }
        
        if (cmd.rightSpeed > 0) {
          digitalWrite(IN3_MOTORES, HIGH); digitalWrite(IN4_MOTORES, LOW);
        } else if (cmd.rightSpeed < 0) {
          digitalWrite(IN3_MOTORES, LOW); digitalWrite(IN4_MOTORES, HIGH);
        }
        
        analogWrite(ENA_MOTORES, abs(cmd.leftSpeed));
        analogWrite(ENB_MOTORES, abs(cmd.rightSpeed));
      }
    }

    // 2. Sensor Ultrasónico (Lectura cada 100ms)
    if (millis() - lastUltraRead > 100) {
      evaluarKillSwitch();
      lastUltraRead = millis();
    }

    // 3. Alertas (Buzzer y LED)
    gestionarEstadoAlerta();

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void TaskProtocol(void *pvParameters) {
  Serial.println("[SYSTEM] TaskProtocol iniciada en Core 0");
  for (;;) {
    if (mensaje_recibido) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      analizarLoteData();
      sistemaExpertoIAElite();
      xSemaphoreGive(dataMutex);

      // Enviar datos al Dashboard
      char jsonResponse[1200];
      snprintf(jsonResponse, sizeof(jsonResponse),
               "{\"heartRate\":%d,\"spo2\":%d,\"temperature\":%.2f,\"pressure\":%.2f,"
               "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"magnitude\":%.2f,"
               "\"estadoGlobal\":\"%s\",\"fallDetected\":%d,\"signalQuality\":%d,"
               "\"nivelRiesgoIA\":%d,\"diagnosticoIA\":\"%s\",\"recomendacionIA\":\"%s\","
               "\"temperatutraAproxIA\":%.2f,\"tipoActividad\":\"%s\",\"altitudEstimada\":%.1f,"
               "\"distanciaObstaculo\":%.1f,\"killSwitchActive\":%d,"
               "\"tendenciaHR\":\"%s\",\"tendenciaSpO2\":\"%s\"}",
               processedData.heartRate, processedData.spo2, processedData.temperature, processedData.pressure,
               processedData.roll, processedData.pitch, processedData.yaw, processedData.magnitude,
               processedData.estadoGlobal, processedData.fallDetected, processedData.signalQuality,
               nivelRiesgoIA, diagnosticoIA, recomendacionIA, temperatutraAproxIA,
               tipoActividad, altitudEstimada, distanciaObstaculo, killSwitchActive ? 1 : 0,
               tendenciaHR, tendenciaSpO2);

      // Lógica de Movimiento IA
      MotorCommand cmd = {0, 0, false};
      if (nivelRiesgoIA == 3) {
        cmd.leftSpeed = 160; cmd.rightSpeed = -160; // Girar para alertar
      } else if (nivelRiesgoIA == 2) {
        cmd.brake = true; // Frenado preventivo
      } else {
        cmd.leftSpeed = 0; cmd.rightSpeed = 0; // Detener motores en estados bajos
      }
      xQueueSend(motorQueue, &cmd, 0);

      events.send(jsonResponse, "message", millis());
      mensaje_recibido = false;
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ==================== SISTEMA DE ALERTAS CODIFICADAS (BUZZER) ====================
void generarPatronAlerta(PatronAlerta patron) {
  static unsigned long ultimoCambio = 0;
  static int paso = 0;
  static bool buzzerEncendido = false;

  if (!alertaActiva || patron == PATRON_NORMAL) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerEncendido = false;
    return;
  }

  unsigned long ahora = millis();
  unsigned long intervalo = 0;
  unsigned long duracionEncendido = 0;

  switch (patron) {
    case PATRON_MONITOREO:
      intervalo = 2000;
      duracionEncendido = 100;
      break;

    case PATRON_ALERTA:
      intervalo = 500;
      duracionEncendido = 150;
      break;

    case PATRON_CRITICO:
      intervalo = 150;
      duracionEncendido = 75;
      break;

    case PATRON_CAIDA:
      intervalo = 400;
      duracionEncendido = 300;
      break;

    case PATRON_TAQUICARDIA:
      intervalo = 250;
      duracionEncendido = 100;
      break;

    case PATRON_BRADICARDIA:
      intervalo = 1200;
      duracionEncendido = 400;
      break;

    case PATRON_HIPOXIA:
      intervalo = 800;
      duracionEncendido = 200;
      break;

    case PATRON_GOLPE_CALOR:
      intervalo = 100;
      duracionEncendido = 50;
      break;

    default:
      digitalWrite(BUZZER_PIN, LOW);
      return;
  }

  if (ahora - ultimoCambio >= (buzzerEncendido ? duracionEncendido : (intervalo - duracionEncendido))) {
    buzzerEncendido = !buzzerEncendido;
    digitalWrite(BUZZER_PIN, buzzerEncendido ? HIGH : LOW);
    ultimoCambio = ahora;
  }
}

// ==================== PROCESAMIENTO ESTADÍSTICO DE LOTES (WINDOW ANALYSIS) ====================
void analizarLoteData() {
  if (batchIndex == 0) return;

  // Acumuladores para promedios
  double sumHR = 0, sumSpO2 = 0, sumTemp = 0, sumPres = 0;
  double sumRoll = 0, sumPitch = 0, sumYaw = 0, sumMag = 0;
  int countValidHR = 0, countValidSpO2 = 0;
  uint8_t fallAny = 0;
  uint32_t latestTimestamp = 0;
  uint8_t avgSignal = 0;

  for (int i = 0; i < batchIndex; i++) {
    if (batchBuffer[i].heartRate > 0) {
      sumHR += batchBuffer[i].heartRate;
      countValidHR++;
    }
    if (batchBuffer[i].spo2 > 0) {
      sumSpO2 += batchBuffer[i].spo2;
      countValidSpO2++;
    }
    sumTemp += batchBuffer[i].temperature;
    sumPres += batchBuffer[i].pressure;
    sumRoll += batchBuffer[i].roll;
    sumPitch += batchBuffer[i].pitch;
    sumYaw += batchBuffer[i].yaw;
    sumMag += batchBuffer[i].magnitude;
    avgSignal += batchBuffer[i].signalQuality;

    if (batchBuffer[i].fallDetected) fallAny = 1;
    if (batchBuffer[i].timestamp > latestTimestamp) latestTimestamp = batchBuffer[i].timestamp;
  }

  // Generar Paquete Procesado (Sintético)
  processedData.timestamp = latestTimestamp;
  processedData.heartRate = (countValidHR > 0) ? (int32_t)(sumHR / countValidHR) : 0;
  processedData.spo2 = (countValidSpO2 > 0) ? (int32_t)(sumSpO2 / countValidSpO2) : 0;
  processedData.temperature = sumTemp / batchIndex;
  processedData.pressure = sumPres / batchIndex;
  processedData.roll = sumRoll / batchIndex;
  processedData.pitch = sumPitch / batchIndex;
  processedData.yaw = sumYaw / batchIndex;
  processedData.magnitude = sumMag / batchIndex;
  processedData.fallDetected = fallAny;
  processedData.signalQuality = avgSignal / batchIndex;

  // Copiar estado global del último paquete
  strncpy(processedData.estadoGlobal, batchBuffer[batchIndex-1].estadoGlobal, 19);

  // Reiniciar lote
  batchIndex = 0;
  fallUrgent = false;
}

// ==================== CÁLCULO DE ALTITUD Y UMBRALES DINÁMICOS ====================
float calcularAltitud(float presionHpa) {
  // Fórmula barométrica simplificada
  return 44330.0 * (1.0 - pow(presionHpa / PRESION_NIVEL_MAR, 0.1903));
}

void actualizarUmbralesDinamicos() {
  altitudEstimada = calcularAltitud(processedData.pressure);

  // Ajuste de umbral SpO2 según altitud
  // A nivel del mar: SpO2 < 92 es alerta
  // A 2500m: SpO2 < 88 es normal, < 82 es alerta
  // A 4000m: SpO2 < 80 es normal, < 75 es alerta

  if (altitudEstimada < 1500) {
    umbralSpO2Dinamico = SPO2_ALERTA;  // 92%
  } else if (altitudEstimada < 2500) {
    umbralSpO2Dinamico = 88;
  } else if (altitudEstimada < 3500) {
    umbralSpO2Dinamico = 84;
  } else {
    umbralSpO2Dinamico = 80;
  }
}

// ==================== FUSIÓN SENSORIAL Y CLASIFICACIÓN DE ACTIVIDAD ====================
const char* clasificarActividad() {
  float magnitud = processedData.magnitude;
  int hr = processedData.heartRate;
  float derivada = bufferGetTendencia(&bufferHR);

  // Correlación BPM-Movimiento
  if (magnitud > UMBRAL_MAGNITUD_ACTIVIDAD) {
    // Movimiento alto detectado
    if (hr > HR_NORMAL_MAX && hr < HR_EJERCICIO_MAX) {
      // BPM elevado con movimiento = Actividad física normal
      return "EJERCICIO_VIGOROSO";
    } else if (hr >= HR_EJERCICIO_MAX) {
      return "EJERCICIO_INTENSO";
    }
    return "MOVIMIENTO_ACTIVO";
  }

  // Movimiento bajo/nulo
  if (magnitud < UMBRAL_MAGNITUD_REPOSO) {
    if (hr > 100 && derivada > 0) {
      // BPM subiendo sin movimiento = TAQUICARDIA EN REPOSO
      return "REPOSO_TAQUICARDICO";
    } else if (hr < 50 && hr > 0) {
      return "REPOSO_BRADICARDICO";
    }
    return "REPOSO_ABSOLUTO";
  }

  return "ACTIVIDAD_LEVE";
}

bool esFalsaAlarmaTaquicardia() {
  const char* actividad = clasificarActividad();
  return (strcmp(actividad, "EJERCICIO_VIGOROSO") == 0 ||
          strcmp(actividad, "EJERCICIO_INTENSO") == 0 ||
          strcmp(actividad, "MOVIMIENTO_ACTIVO") == 0);
}

// ==================== SISTEMA EXPERTO IA ELITE (FUZZY LOGIC + VFC) ====================

float calcularVFC() {
    if (batchIndex < 5) return 0.0f;
    float sum = 0, mean, variance = 0;
    for (int i = 0; i < batchIndex; i++) sum += batchBuffer[i].heartRate;
    mean = sum / (float)batchIndex;
    for (int i = 0; i < batchIndex; i++) variance += pow((float)batchBuffer[i].heartRate - mean, 2);
    return sqrt(variance / (float)batchIndex); 
}

void sistemaExpertoIAElite() {
    actualizarUmbralesDinamicos();

    if (processedData.heartRate > 0) bufferPush(&bufferHR, (float)processedData.heartRate);
    if (processedData.spo2 > 0) bufferPush(&bufferSpO2, (float)processedData.spo2);
    bufferPush(&bufferTemp, processedData.temperature);

    derivadaHR = bufferGetTendencia(&bufferHR);
    derivadaSpO2 = bufferGetTendencia(&bufferSpO2);
    
    strcpy(tipoActividad, clasificarActividad());
    float vfcIndex = calcularVFC();

    float bpm = (float)processedData.heartRate;
    float spo2 = (float)processedData.spo2;
    float temp = processedData.temperature;

    float mBPM_Low = trapmf(bpm, 0, 0, 45, 60);
    float mBPM_Normal = trimf(bpm, 50, 75, 100);
    float mBPM_High = trapmf(bpm, 90, 120, 220, 220);

    float mSPO2_Critical = trapmf(spo2, 0, 0, 80, 88);
    float mSPO2_Alert = trimf(spo2, 85, 90, 94);
    float mSPO2_Stable = trapmf(spo2, 92, 96, 100, 100);

    float mTEMP_Fiebre = trapmf(temp, 37.5, 38.5, 39.5, 40.5);
    float mTEMP_Golpe = trapmf(temp, 39.5, 41.0, 45.0, 45.0);

    float riesgoTotal = 0.0f;
    riesgoTotal += mBPM_High * 0.4f;
    riesgoTotal += mBPM_Low * 0.3f;
    riesgoTotal += mSPO2_Critical * 0.5f;
    riesgoTotal += mSPO2_Alert * 0.2f;
    riesgoTotal += mTEMP_Golpe * 0.4f;
    
    if (vfcIndex < 2.0 && bpm > 100) riesgoTotal += 0.15f;
    if (processedData.fallDetected) riesgoTotal = 1.0f;
    if (riesgoTotal > 1.0f) riesgoTotal = 1.0f;

    memset(diagnosticoIA, 0, sizeof(diagnosticoIA));
    memset(recomendacionIA, 0, sizeof(recomendacionIA));

    if (riesgoTotal >= 0.75f) {
        nivelRiesgoIA = 3;
        estadoPaciente = ESTADO_CRITICO;
        strcat(diagnosticoIA, "CRISIS MULTISISTÉMICA. ");
        strcat(recomendacionIA, "INTERVENCIÓN INMEDIATA. ");
        patronAlertaActual = processedData.fallDetected ? PATRON_CAIDA : PATRON_CRITICO;
    } else if (riesgoTotal >= 0.45f) {
        nivelRiesgoIA = 2;
        estadoPaciente = ESTADO_ALERTA;
        strcat(diagnosticoIA, "INESTABILIDAD DETECTADA. ");
        strcat(recomendacionIA, "MONITOREO INTENSIVO. ");
        patronAlertaActual = PATRON_ALERTA;
    } else if (riesgoTotal >= 0.20f) {
        nivelRiesgoIA = 1;
        estadoPaciente = ESTADO_MONITOREO;
        strcat(diagnosticoIA, "ANOMALÍA LEVE. ");
        strcat(recomendacionIA, "OBSERVACIÓN PREVENTIVA. ");
        patronAlertaActual = PATRON_MONITOREO;
    } else {
        nivelRiesgoIA = 0;
        estadoPaciente = ESTADO_NORMAL;
        strcat(diagnosticoIA, "SISTEMAS NOMINALES.");
        strcat(recomendacionIA, "CONTINUAR MONITOREO.");
        patronAlertaActual = PATRON_NORMAL;
    }

    if (nivelRiesgoIA >= 2) alertaDeEmergencia(20);

    if (nivelRiesgoIA >= 2 && (millis() - lastWhatsAppTime > WHATSAPP_COOLDOWN)) {
      WhatsAppAlert alert;
      alert.riskLevel = nivelRiesgoIA;
      alert.hr = (int)bpm;
      alert.spo2 = (int)spo2;
      alert.temp = temperatutraAproxIA;
      alert.fall = (bool)processedData.fallDetected;
      strncpy(alert.diagnosis, diagnosticoIA, 127);
      strncpy(alert.recommendation, recomendacionIA, 127);
      strncpy(alert.state, ESTADO_STR[estadoPaciente], 19);
      xQueueSend(whatsappQueue, &alert, 0);
      lastWhatsAppTime = millis();
    }
}

void alertaDeEmergencia(int duracionSegundos) {
  duracionAlertaMs = duracionSegundos * 1000UL;
  tiempoInicioAlerta = millis();
  alertaActiva = true;
  Serial.printf("[ALERTA] Iniciada por %d segundos.\n", duracionSegundos);
}

void gestionarEstadoAlerta() {
  if (alertaActiva) {
    if (millis() - tiempoInicioAlerta < duracionAlertaMs) {
      // LED parpadeante
      if ((millis() / 250) % 2 == 0) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
      // Buzzer con patrón codificado
      generarPatronAlerta((PatronAlerta)patronAlertaActual);

      // --- ALERTA DE MOVIMIENTO: Girar en círculos si es estado CRÍTICO ---
      if (nivelRiesgoIA == 3) {
        motoresGirarDerecha(160); // Velocidad moderada para alerta visual
      }
    } else {
      alertaActiva = false;
      digitalWrite(LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      motoresDetener(); // Detener movimiento al finalizar la alerta
      Serial.println("[ALERTA] Finalizada.");
    }
  }
}

// Callback de ESP-NOW
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len == sizeof(TelemetryPacket)) {
    TelemetryPacket packet;
    memcpy(&packet, data, sizeof(packet));

    // Guardar en buffer de lote
    if (batchIndex < IA_BATCH_SIZE) {
      batchBuffer[batchIndex] = packet;
      
      // Si hay caída, marcar como urgente
      if (packet.fallDetected) {
        fallUrgent = true;
        mensaje_recibido = true;
      }

      batchIndex++;

      // Si llegamos al límite del lote
      if (batchIndex == IA_BATCH_SIZE) {
        mensaje_recibido = true;
      }
    }
  } else {
    Serial.printf("[WARNING] Paquete descartado. Tamaño: %d (Esperado: %d)\n",
                   len, sizeof(TelemetryPacket));
  }
}

// ==================== GESTIÓN ASÍNCRONA WIFI Y WHATSAPP ====================
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("\n[WIFI] STA Conectado. IP: ");
      Serial.println(WiFi.localIP());
      uint8_t primaryChan;
      wifi_second_chan_t secondChan;
      esp_wifi_get_channel(&primaryChan, &secondChan);
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(primaryChan, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);
      Serial.printf("[WIFI] Canal sincronizado: %d\n", primaryChan);
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("\n[WIFI] STA Desconectado. Reintentando...");
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      break;
    default:
      break;
  }
}

void TaskWhatsApp(void *pvParameters) {
  WhatsAppAlert alert;
  while (1) {
    if (xQueueReceive(whatsappQueue, &alert, portMAX_DELAY)) {
      if (WiFi.status() == WL_CONNECTED) {
        WiFiClientSecure client;
        HTTPClient http;

        client.setInsecure();

        String msg = "🚨 *ALERTA MÉDICA CRÍTICA - ALMA ELITE* 🚨\n\n";
        msg += "*ESTADO:* " + String(alert.state) + "\n";
        msg += "*DIAGNÓSTICO:* " + String(alert.diagnosis) + "\n";
        msg += "*RECOMENDACIÓN:* " + String(alert.recommendation) + "\n\n";
        msg += "❤️ *BPM:* " + String(alert.hr) + "\n";
        msg += "🩸 *SpO2:* " + String(alert.spo2) + "%\n";
        msg += "🌡️ *TEMP EST:* " + String(alert.temp, 1) + "°C\n";
        msg += "⚠️ *CAÍDA:* " + String(alert.fall ? "SÍ" : "NO");

        String url = "https://api.callmebot.com/whatsapp.php?phone=" + String(CALLMEBOT_PHONE) +
                     "&apikey=" + String(CALLMEBOT_API_KEY) +
                     "&text=" + urlEncode(msg);

        Serial.println("[WHATSAPP] Enviando a CallMeBot...");
        http.begin(client, url);
        int httpResponseCode = http.GET();

        if (httpResponseCode > 0) {
          Serial.printf("[WHATSAPP] Enviado. HTTP: %d\n", httpResponseCode);
        } else {
          Serial.printf("[WHATSAPP] Error: %s\n", http.errorToString(httpResponseCode).c_str());
        }
        http.end();
      } else {
        Serial.println("[WHATSAPP] Sin conexión.");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1. Inicializar Sincronización
  dataMutex = xSemaphoreCreateMutex();
  motorQueue = xQueueCreate(10, sizeof(MotorCommand));
  whatsappQueue = xQueueCreate(5, sizeof(WhatsAppAlert));

  // 2. Inicializar Búferes
  bufferInit(&bufferHR);
  bufferInit(&bufferSpO2);
  bufferInit(&bufferTemp);

  // 3. Configurar Hardware
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  motoresInit();

  // 4. Comunicaciones (Core 0)
  WiFi.mode(WIFI_AP_STA);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP("ALMA_ELITE_DASHBOARD", "12345678", 1);

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.addHandler(&events);
  server.begin();

  // 5. Lanzamiento de Tareas Multi-Core
  xTaskCreatePinnedToCore(TaskHardware, "HardwareTask", 4096, NULL, 3, &TaskHardwareHandle, 1); // Core 1
  xTaskCreatePinnedToCore(TaskProtocol, "ProtocolTask", 8192, NULL, 1, &TaskProtocolHandle, 0); // Core 0
  xTaskCreatePinnedToCore(TaskWhatsApp, "WhatsAppTask", 4096, NULL, 1, NULL, 0); // Core 0

  Serial.println(">>> ALMA ELITE SYSTEM READY <<<");
}

void loop() {
  // El sistema opera bajo FreeRTOS Tasks. loop() queda liberado.
  vTaskDelete(NULL); 
}