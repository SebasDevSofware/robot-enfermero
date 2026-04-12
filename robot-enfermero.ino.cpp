#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <UrlEncode.h>

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

// Dashboard HTML/CSS/JS
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ALMA | DIGITAL MEDICAL DASHBOARD</title>
    <style>
        body { background: #0a0a0a; color: #00ffcc; font-family: 'Courier New', monospace; margin: 0; padding: 20px; text-transform: uppercase; overflow-x: hidden; }
        .header { border-bottom: 2px solid #00ffcc; padding-bottom: 10px; margin-bottom: 20px; letter-spacing: 4px; display: flex; justify-content: space-between; }
        .grid { display: grid; gap: 15px; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); }
        .panel { border: 1px solid #00ffcc; padding: 15px; background: rgba(0, 255, 204, 0.05); box-shadow: 0 0 10px rgba(0, 255, 204, 0.1); position: relative; }
        .panel-title { font-size: 0.9em; opacity: 0.7; margin-bottom: 10px; border-left: 3px solid #00ffcc; padding-left: 8px; }
        .val-large { font-size: 3.5em; font-weight: bold; color: #00ffcc; text-shadow: 0 0 15px rgba(0, 255, 204, 0.4); }
        .unit { font-size: 0.4em; vertical-align: middle; opacity: 0.6; }
        .status-panel { grid-column: 1 / -1; padding: 25px; text-align: center; border-width: 2px; transition: all 0.4s ease; }
        .risk-0 { border-color: #00ffcc; color: #00ffcc; }
        .risk-1 { border-color: #ff9900; color: #ff9900; }
        .risk-2 { border-color: #ff9900; color: #ff9900; box-shadow: 0 0 20px rgba(255, 153, 0, 0.3); }
        .risk-3 { border-color: #ff3333; color: #ff3333; box-shadow: 0 0 30px rgba(255, 51, 51, 0.5); animation: blink 1s infinite; }
        @keyframes blink { 0%, 100% { opacity: 1; } 50% { opacity: 0.6; } }
        canvas { width: 100%; height: 90px; margin-top: 15px; border-top: 1px solid rgba(0, 255, 204, 0.2); }
        .meta { font-size: 0.8em; margin-top: 8px; color: #00ffcc; opacity: 0.8; }
        #fall-alert { color: #ff3333; font-weight: bold; border: 1px solid #ff3333; padding: 10px; margin-top: 15px; display: none; background: rgba(255, 51, 51, 0.1); }
    </style>
</head>
<body>
    <div class="header">
        <span>ALMA_RECEIVER_CORE_V1</span>
        <span id="uptime">SYSTEM_ACTIVE</span>
    </div>

    <div class="grid">
        <div id="risk-box" class="panel status-panel risk-0">
            <div class="panel-title">IA_DIAGNOSTIC_ENGINE</div>
            <div id="diag" style="font-size: 2em; margin-bottom: 10px;">ESPERANDO TELEMETRÍA...</div>
            <div id="reco" style="font-size: 1.1em; opacity: 0.9;">SINCRONIZANDO CON SENSOR REMOTO</div>
            <div id="risk-lv" style="margin-top: 20px; font-weight: bold; font-size: 1.3em;">NIVEL_RIESGO: -</div>
        </div>

        <div class="panel">
            <div class="panel-title">HEART_RATE (BPM)</div>
            <div class="val-large"><span id="hr">--</span><span class="unit"> BPM</span></div>
            <canvas id="hrChart"></canvas>
        </div>

        <div class="panel">
            <div class="panel-title">BLOOD_OXYGEN (SPO2)</div>
            <div class="val-large"><span id="spo2">--</span><span class="unit"> %</span></div>
            <canvas id="spo2Chart"></canvas>
        </div>

        <div class="panel">
            <div class="panel-title">ESTIMATED_BODY_TEMP</div>
            <div class="val-large"><span id="temp-ia">--.-</span><span class="unit"> °C</span></div>
            <div class="meta">AMBIENT: <span id="temp-amb">--.-</span>°C</div>
        </div>

        <div class="panel">
            <div class="panel-title">IMU_POSITION_MATRIX</div>
            <div class="meta">ROLL: <span id="roll">0</span>° | PITCH: <span id="pitch">0</span>°</div>
            <div class="meta">YAW: <span id="yaw">0</span>° | ACCEL: <span id="mag">0.0</span>G</div>
            <div id="fall-alert">!!! CRITICAL: FALL_DETECTED !!!</div>
        </div>

        <div class="panel">
            <div class="panel-title">ENVIRONMENT_&_SIGNAL</div>
            <div class="meta">PRESSURE: <span id="pres">0</span> hPa</div>
            <div class="meta">SIGNAL_QUALITY: <span id="sig">0</span>/100</div>
            <div class="meta">GLOBAL_STATE: <span id="status">INIT</span></div>
        </div>
    </div>

    <script>
        const hrH = [], spH = [];
        const maxP = 40;

        function draw(id, data, color, min, max) {
            const c = document.getElementById(id);
            const ctx = c.getContext('2d');
            ctx.clearRect(0, 0, c.width, c.height);
            ctx.strokeStyle = color;
            ctx.lineWidth = 2;
            ctx.beginPath();
            const dx = c.width / (maxP - 1);
            data.forEach((v, i) => {
                const y = c.height - ((v - min) / (max - min) * c.height);
                if(i===0) ctx.moveTo(0, y); else ctx.lineTo(i * dx, y);
            });
            ctx.stroke();
        }

        const source = new EventSource('/events');
        source.onmessage = function(e) {
            const d = JSON.parse(e.data);
            document.getElementById('hr').innerText = d.heartRate;
            document.getElementById('spo2').innerText = d.spo2;
            document.getElementById('temp-ia').innerText = d.temperatutraAproxIA.toFixed(1);
            document.getElementById('temp-amb').innerText = d.temperature.toFixed(1);
            document.getElementById('roll').innerText = d.roll.toFixed(0);
            document.getElementById('pitch').innerText = d.pitch.toFixed(0);
            document.getElementById('yaw').innerText = d.yaw.toFixed(0);
            document.getElementById('mag').innerText = d.magnitude.toFixed(2);
            document.getElementById('pres').innerText = d.pressure.toFixed(1);
            document.getElementById('sig').innerText = d.signalQuality;
            document.getElementById('status').innerText = d.estadoGlobal;
            document.getElementById('diag').innerText = d.diagnosticoIA;
            document.getElementById('reco').innerText = d.recomendacionIA;
            
            const rL = ["ESTABLE", "OBSERVACIÓN", "ADVERTENCIA", "CRÍTICO"];
            document.getElementById('risk-lv').innerText = "NIVEL_RIESGO: " + rL[d.nivelRiesgoIA];
            document.getElementById('risk-box').className = "panel status-panel risk-" + d.nivelRiesgoIA;
            document.getElementById('fall-alert').style.display = d.fallDetected ? "block" : "none";

            hrH.push(d.heartRate); if(hrH.length > maxP) hrH.shift();
            spH.push(d.spo2); if(spH.length > maxP) spH.shift();
            draw('hrChart', hrH, '#00ffcc', 40, 160);
            draw('spo2Chart', spH, '#ff3333', 70, 100);
        };
    </script>
</body>
</html>
)rawliteral";

// ==================== VARIABLES PINES ====================

#define BUZZER_PIN 18
#define SERVO_PIN 5
#define TRIG_PIN 17
#define ECHO_PIN 16
#define LED_PIN 23

// Motor A
#define IN1_MOTORES 34
#define IN2_MOTORES 35
#define ENA_MOTORES 4
// Motor B
#define IN3_MOTORES 32
#define IN4_MOTORES 33
#define ENB_MOTORES 2

// ==================== UMBRALES MÉDICOS ====================
#define ZONA_PELIGRO_HR_MIN 50
#define ZONA_PELIGRO_HR_MAX 125
#define ZONA_CRITICA_HR_MIN 40
#define ZONA_CRITICA_HR_MAX 150

#define ZONA_PELIGRO_SPO2 92
#define ZONA_CRITICA_SPO2 88

#define UMBRAL_FIEBRE 38.0
#define UMBRAL_HIPOTERMIA 35.0

typedef struct __attribute__((packed)) {
  uint32_t timestamp;
  int32_t heartRate;
  int32_t spo2;
  float temperature; // Temperatura ambiental
  float pressure;
  float roll;
  float pitch;
  float yaw;
  float magnitude;
  char estadoGlobal[20];
  uint8_t fallDetected;
  uint8_t signalQuality;
} TelemetryPacket;

TelemetryPacket incomingData; // Datos recibidos de emisor (ALMA)
volatile bool mensaje_recibido = false;

// Variables globales para almacenar el resultado de la IA en el receptor
uint8_t nivelRiesgoIA = 0;
char diagnosticoIA[128] = "";
char recomendacionIA[128] = "";
float temperatutraAproxIA = 36.5;

// ==================== CONTROL DE ALARMA ASÍNCRONA ====================
unsigned long tiempoInicioAlerta = 0;
unsigned long duracionAlertaMs = 0;
bool alertaActiva = false;

void alertaDeEmergencia(int duracionSegundos) {
  duracionAlertaMs = duracionSegundos * 1000UL;
  tiempoInicioAlerta = millis();
  alertaActiva = true;
  Serial.printf("\n[SISTEMA] Alarma de emergencia iniciada por %d segundos.\n", duracionSegundos);
}

// Mantiene el estado de la alerta ejecutándose asíncronamente en el loop
void gestionarEstadoAlerta() {
  if (alertaActiva) {
    if (millis() - tiempoInicioAlerta < duracionAlertaMs) {
      // Crea un patrón de pitido y parpadeo de 500ms (oscilación)
      if ((millis() / 500) % 2 == 0) {
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(BUZZER_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
        digitalWrite(BUZZER_PIN, LOW);
      }
    } else {
      alertaActiva = false;
      digitalWrite(LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      Serial.println("\n[SISTEMA] Alarma de emergencia finalizada.");
    }
  }
}

// --- MÓDULO DE INTELIGENCIA DE BORDE (EDGE IA) ---
void analizarContextoMedico() {
  String conclusion = "ESTABLE";
  bool alertaCritica = false;

  if (incomingData.signalQuality == 0) {
    strcpy(incomingData.estadoGlobal, "DATO NO FIABLE");
    return;
  }

  if (incomingData.temperature > 35.0 && incomingData.heartRate > 100) {
    conclusion = "RIESGO GOLPE CALOR";
    alertaCritica = true;
  }

  bool esAltaMontana = (incomingData.pressure < 800.0);

  if (incomingData.spo2 < 94) {
    if (esAltaMontana && incomingData.spo2 >= 88) {
      conclusion = "HIPOXIA AMBIENTAL"; 
    } else if (incomingData.spo2 < 88) {
      conclusion = "HIPOXIA CRITICA";   
      alertaCritica = true;
    } else {
      conclusion = "FALLO RESPIRATORIO"; 
      alertaCritica = true;
    }
  }

  if (incomingData.heartRate > 120 && !alertaCritica) {
    conclusion = "TAQUICARDIA";
  } else if (incomingData.heartRate < 50 && incomingData.heartRate > 0) {
    conclusion = "BRADICARDIA";
  }

  if (incomingData.fallDetected) {
    conclusion = "PACIENTE CAIDO";
  }

  strncpy(incomingData.estadoGlobal, conclusion.c_str(), 19);
  incomingData.estadoGlobal[19] = '\0';
}

// ==================== SISTEMA EXPERTO IA (FUZZY LOGIC) ====================
void sistemaExpertoIA() {
  float riesgoHR = 0.0f, riesgoSpO2 = 0.0f, riesgoTemp = 0.0f, riesgoMov = 0.0f;
  float riesgoTotal = 0.0f;

  // === LÓGICA DIFUSA PARA FRECUENCIA CARDIACA ===
  if (incomingData.heartRate > 0) {
    if (incomingData.heartRate < ZONA_PELIGRO_HR_MIN) {
      riesgoHR = 0.8f + (ZONA_PELIGRO_HR_MIN - incomingData.heartRate) / 20.0f;
      if (riesgoHR > 1.0f) riesgoHR = 1.0f;
    } else if (incomingData.heartRate > ZONA_PELIGRO_HR_MAX) {
      riesgoHR = 0.8f + (incomingData.heartRate - ZONA_PELIGRO_HR_MAX) / 30.0f;
      if (riesgoHR > 1.0f) riesgoHR = 1.0f;
    } else if (incomingData.heartRate < ZONA_CRITICA_HR_MIN || incomingData.heartRate > ZONA_CRITICA_HR_MAX) {
      riesgoHR = 0.4f;
    } else {
      riesgoHR = 0.0f;
    }
  }

  // === LÓGICA DIFUSA PARA SATURACIÓN ===
  if (incomingData.spo2 > 0) {
    if (incomingData.spo2 <= ZONA_CRITICA_SPO2) {
      riesgoSpO2 = 1.0f;
    } else if (incomingData.spo2 <= ZONA_PELIGRO_SPO2) {
      riesgoSpO2 = 0.7f;
    } else if (incomingData.spo2 <= 94) {
      riesgoSpO2 = 0.3f;
    }
  }

  // === LÓGICA DIFUSA PARA TEMPERATURA ===
  if (incomingData.temperature > 0) {
    if (incomingData.temperature >= UMBRAL_FIEBRE) {
      riesgoTemp = 0.7f + (incomingData.temperature - UMBRAL_FIEBRE) / 2.0f;
      if (riesgoTemp > 1.0f) riesgoTemp = 1.0f;
    } else if (incomingData.temperature <= UMBRAL_HIPOTERMIA) {
      riesgoTemp = 0.8f + (UMBRAL_HIPOTERMIA - incomingData.temperature) / 3.0f;
      if (riesgoTemp > 1.0f) riesgoTemp = 1.0f;
    }
  }

  // === LÓGICA DIFUSA PARA MOVIMIENTO/CAÍDA ===
  if (incomingData.fallDetected) {
    riesgoMov = 1.0f;
  } else if (incomingData.magnitude > 3.5f) {
    riesgoMov = 0.5f;
  } else if (abs(incomingData.pitch) > 70) {
    riesgoMov = 0.4f;
  }

  // === CALCULO APOXIMADO DE TEMPERATURA CORPORAL ===
  float tempCalculada = 36.5f; 
  if (incomingData.heartRate > 80 && !incomingData.fallDetected) {
    float excesoHR = (float)incomingData.heartRate - 80.0f;
    tempCalculada += (excesoHR / 12.0f); 
  }
  if (incomingData.magnitude > 2.5f) tempCalculada -= 0.6f; 
  if (incomingData.temperature > 38.0f) tempCalculada += 0.4f;
  temperatutraAproxIA = tempCalculada;

  // === CÁLCULO DE RIESGO TOTAL (PONDERADO) ===
  riesgoTotal = (riesgoHR * 0.35f) + (riesgoSpO2 * 0.35f) + (riesgoTemp * 0.2f) + (riesgoMov * 0.1f);
  if (riesgoTotal > 1.0f) riesgoTotal = 1.0f;

  // === DETERMINAR NIVEL DE RIESGO ===
  if (riesgoTotal >= 0.7f) nivelRiesgoIA = 3;      // ROJO - CRÍTICO
  else if (riesgoTotal >= 0.4f) nivelRiesgoIA = 2; // NARANJA - ALERTA
  else if (riesgoTotal >= 0.15f) nivelRiesgoIA = 1;// AMARILLO - MONITOREO
  else nivelRiesgoIA = 0;                          // VERDE - NORMAL

  // === GENERAR DIAGNÓSTICO Y RECOMENDACIÓN ===
  memset(diagnosticoIA, 0, sizeof(diagnosticoIA));
  memset(recomendacionIA, 0, sizeof(recomendacionIA));

  if (nivelRiesgoIA >= 2 || incomingData.fallDetected) {
    if (riesgoHR > 0.7f) {
      if (incomingData.heartRate < ZONA_PELIGRO_HR_MIN) strcat(diagnosticoIA, "BRADICARDIA SEVERA. ");
      else strcat(diagnosticoIA, "TAQUICARDIA SEVERA. ");
      strcat(recomendacionIA, "REQUIERE ATENCIÓN INMEDIATA. ");
    }
    if (riesgoSpO2 > 0.7f) {
      strcat(diagnosticoIA, "HIPOXEMIA CRÍTICA. ");
      strcat(recomendacionIA, "ADMINISTRAR OXÍGENO. ");
    }
    if (riesgoTemp > 0.7f) {
      if (incomingData.temperature >= UMBRAL_FIEBRE) strcat(diagnosticoIA, "HIPERTERMIA SEVERA. ");
      else strcat(diagnosticoIA, "HIPOTERMIA SEVERA. ");
      strcat(recomendacionIA, "CONTROLAR TEMPERATURA CORPORAL. ");
    }
    if (incomingData.fallDetected) {
      strcat(diagnosticoIA, "CAÍDA DETECTADA. ");
      strcat(recomendacionIA, "ACTIVAR PROTOCOLO DE ASISTENCIA. ");
    }
    
    // Activa la alarma física de emergencia si las condiciones son serias o críticas
    // Si la alarma ya está corriendo, alertaDeEmergencia simplemente reiniciará el temporizador
    alertaDeEmergencia(10); // Activa durante 10 segundos
    
    // --- GESTIÓN DE ALERTA WHATSAPP (Anti-Spam 60s) ---
    if (millis() - lastWhatsAppTime > WHATSAPP_COOLDOWN) {
      WhatsAppAlert alert;
      alert.riskLevel = nivelRiesgoIA;
      strncpy(alert.diagnosis, diagnosticoIA, 127);
      strncpy(alert.recommendation, recomendacionIA, 127);
      alert.hr = incomingData.heartRate;
      alert.spo2 = incomingData.spo2;
      alert.temp = temperatutraAproxIA;
      alert.fall = (incomingData.fallDetected > 0);
      strncpy(alert.state, incomingData.estadoGlobal, 19);

      if (xQueueSend(whatsappQueue, &alert, 0) == pdPASS) {
        lastWhatsAppTime = millis();
        Serial.println("[SISTEMA] Alerta WhatsApp encolada correctamente.");
      }
    }
    
  } else if (nivelRiesgoIA == 1) {
    if (riesgoHR > 0.2f) strcat(diagnosticoIA, "ALTERACIÓN CARDÍACA LEVE. ");
    if (riesgoSpO2 > 0.2f) strcat(diagnosticoIA, "DESATURACIÓN LEVE. ");
    if (riesgoTemp > 0.2f) strcat(diagnosticoIA, "TEMP ANORMAL. ");
    strcat(recomendacionIA, "MONITORIZAR CONSTANTES. ");
  } else {
    strcat(diagnosticoIA, "PARÁMETROS NORMALES.");
    strcat(recomendacionIA, "MANTENER MONITOREO.");
  }
}

// Callback de ESP-NOW
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len == sizeof(TelemetryPacket)) {
    memcpy(&incomingData, data, sizeof(incomingData));
    mensaje_recibido = true;
  } else {
    Serial.printf("[WARNING] Paquete descartado. Tamaño anómalo: %d bytes (Esperados: %d)\n", len, sizeof(TelemetryPacket));
  }
}

// ==================== GESTIÓN ASÍNCRONA WIFI Y WHATSAPP ====================

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("\n[WIFI] STA Conectado. IP: ");
      Serial.println(WiFi.localIP());
      // Sincronizar canal de ESP-NOW con el de la STA
      uint8_t primaryChan;
      wifi_second_chan_t secondChan;
      esp_wifi_get_channel(&primaryChan, &secondChan);
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(primaryChan, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);
      Serial.printf("[WIFI] Canal sincronizado a: %d para coexistencia SoftAP/ESP-NOW\n", primaryChan);
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
        
        client.setInsecure(); // No verificar certificados para ahorrar recursos
        
        String msg = "🚨 *ALERTA MÉDICA CRÍTICA - ALMA* 🚨\n\n";
        msg += "*ESTADO:* " + String(alert.state) + "\n";
        msg += "*DIAGNÓSTICO:* " + String(alert.diagnosis) + "\n";
        msg += "*RECO:* " + String(alert.recommendation) + "\n\n";
        msg += "❤️ *BPM:* " + String(alert.hr) + "\n";
        msg += "🩸 *SPO2:* " + String(alert.spo2) + "%\n";
        msg += "🌡️ *TEMP IA:* " + String(alert.temp, 1) + "°C\n";
        msg += "⚠️ *CAÍDA:* " + String(alert.fall ? "SÍ" : "NO");

        String url = "https://api.callmebot.com/whatsapp.php?phone=" + String(CALLMEBOT_PHONE) +
                     "&apikey=" + String(CALLMEBOT_API_KEY) +
                     "&text=" + urlEncode(msg);

        Serial.println("[WHATSAPP] Enviando alerta a CallMeBot...");
        http.begin(client, url);
        int httpResponseCode = http.GET();
        
        if (httpResponseCode > 0) {
          Serial.printf("[WHATSAPP] Mensaje enviado. Respuesta: %d\n", httpResponseCode);
        } else {
          Serial.printf("[WHATSAPP] Error HTTP: %s\n", http.errorToString(httpResponseCode).c_str());
        }
        http.end();
      } else {
        Serial.println("[WHATSAPP] Omitido: Sin conexión a internet.");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1. Wi-Fi en Modo Híbrido (AP + Estación para coexistencia con ESP-NOW)
  WiFi.mode(WIFI_AP_STA);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(WIFI_SSID, WIFI_PASS); // Conexión asíncrona no bloqueante
  
  // Configuración de SoftAP (Portal Cautivo Offline)
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP("ALMA_DASHBOARD", "12345678", 1); // Canal 1 inicial

  // Forzar canal 1 para ESP-NOW inicialmente
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.println("\n--- SERVIDOR ALMA INICIADO ---");
  Serial.print("IP del Dashboard: "); Serial.println(WiFi.softAPIP());

  // 2. Inicialización de ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // 3. Configuración de Rutas Web Asíncronas
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  
  server.addHandler(&events);
  server.begin();

  // 4. Inicialización de Multihilo (WhatsApp)
  whatsappQueue = xQueueCreate(5, sizeof(WhatsAppAlert));
  xTaskCreatePinnedToCore(TaskWhatsApp, "WhatsAppTask", 8192, NULL, 1, NULL, 1);

  // Configuración de pines para la alarma
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT); 
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.println("Dashboard Médico y Receptor listos...");
}

void loop() {
  // 0. Ejecutar lógica asíncrona de la alarma
  gestionarEstadoAlerta();

  if (mensaje_recibido) {
    // 1. Procesar los datos de manera secuencial y segura fuera de la interrupción
    analizarContextoMedico();
    sistemaExpertoIA();
    
    // 2. Empaquetar y enviar datos vía SSE (Dashboard Digital)
    char jsonResponse[1024];
    snprintf(jsonResponse, sizeof(jsonResponse),
             "{\"heartRate\":%d,\"spo2\":%d,\"temperature\":%.2f,\"pressure\":%.2f,"
             "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"magnitude\":%.2f,"
             "\"estadoGlobal\":\"%s\",\"fallDetected\":%d,\"signalQuality\":%d,"
             "\"nivelRiesgoIA\":%d,\"diagnosticoIA\":\"%s\",\"recomendacionIA\":\"%s\","
             "\"temperatutraAproxIA\":%.2f}",
             incomingData.heartRate, incomingData.spo2, incomingData.temperature, incomingData.pressure,
             incomingData.roll, incomingData.pitch, incomingData.yaw, incomingData.magnitude,
             incomingData.estadoGlobal, incomingData.fallDetected, incomingData.signalQuality,
             nivelRiesgoIA, diagnosticoIA, recomendacionIA, temperatutraAproxIA);
    
    events.send(jsonResponse, "message", millis());

    // 3. Reporte en consola

    if (incomingData.fallDetected) Serial.println("!!! ALERTA DE IMPACTO DETECTADA !!!");
    
    Serial.println("\n=== ANALISIS CLINICO ALMA ===");
    Serial.printf("BIO -> BPM: %d | SpO2: %d%% | Temp Corp Aprox: %.1f C\n", 
                  incomingData.heartRate, incomingData.spo2, temperatutraAproxIA);
    
    Serial.printf("ENV -> Temp Amb: %.1f C | Presion: %.1f hPa\n", 
                  incomingData.temperature, incomingData.pressure);
                  
    Serial.printf("CTX -> ESTADO RAPIDO: %s\n", incomingData.estadoGlobal);

    Serial.println("\n--- REPORTE PROFUNDO: LOGICA DIFUSA ---");
    Serial.printf("Nivel de Riesgo (0-3): %d\n", nivelRiesgoIA);
    Serial.printf("Diagnóstico: %s\n", diagnosticoIA);
    Serial.printf("Recomendación: %s\n", recomendacionIA);
    Serial.printf("\n--- HORA DEL ENVIO: %d ---", incomingData.timestamp);
    
    // 3. Lógica de actuación basada en los resultados de la IA
    Serial.println("\n--- COMANDOS DE ACTUADOR ---");
    if (nivelRiesgoIA >= 2) {
        Serial.println("ACCIÓN: Activando sirena de emergencia a través del módulo de voz (DFPlayer Pro).");
        Serial.println("NAVEGACIÓN: Deteniendo motores DC y calculando ruta hacia el punto de extracción.");
    }
    
    if (strcmp(incomingData.estadoGlobal, "RIESGO GOLPE CALOR") == 0) {
        Serial.println("ACCIÓN: Desplegando kit de dispensación (Servos MG996R) para entregar sales de rehidratación oral.");
    } 
    else if (strcmp(incomingData.estadoGlobal, "HIPOXIA AMBIENTAL") == 0) {
        Serial.println("AVISO: Paciente en altura según datos barométricos. SpO2 bajo justificado.");
    }

    Serial.println("=============================");
    
    mensaje_recibido = false; 
  }
}
