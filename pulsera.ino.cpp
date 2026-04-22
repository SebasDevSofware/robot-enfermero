// ===================================================================
// |                                                                 |
// |  BIOCHRONORITMIC  - Asistente Logístico de Monitoreo Avanzado   |    |                                                                 |
// |                                                                 |
// |  Características:                                               |
// |  - Multi-tarea optimizada (Core 0: Biometría / Core 1: Mov.)    |
// |  - Filtro Madgwick para orientación absoluta                    |
// |  - Detección de caídas con confirmación por ángulo              |
// |  - Buffer circular PPG sin pérdida de muestras                  |
// |  - Filtro de outliers médicos (sanity check)                    |
// |  - Modo Low Power con wake-on-motion                            |
// |  - Watchdog por software con reinicio I2C                       |
// |  - Estructura binaria para telemetría LoRa/BLE                  |
// |  - Barra de calidad de señal en OLED                            |
// |  - Calibración dinámica de orientación                          |
// |                                                                 |
// ===================================================================

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include "MPU9250.h"
#include <WiFi.h>
#include <esp_now.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

// ==================== CONFIGURACIÓN DE RED (MODO AP) ====================
#define AP_SSID "BIOCHORNORITMIC_LIVE_DATA"
#define AP_PASS "12345678"
#define WEB_SERVER_PORT 80

AsyncWebServer server(WEB_SERVER_PORT);
AsyncEventSource events("/events");

// Dashboard HTML/CSS/JS
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ALMA | BRACELET TELEMETRY</title>
    <style>
        body { background: #0a0a0a; color: #00ffcc; font-family: 'Courier New', monospace; margin: 0; padding: 15px; text-transform: uppercase; }
        .header { border-bottom: 2px solid #00ffcc; padding-bottom: 10px; margin-bottom: 20px; display: flex; justify-content: space-between; font-weight: bold; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 15px; }
        .panel { border: 1px solid rgba(0, 255, 204, 0.3); padding: 15px; background: rgba(0, 20, 20, 0.5); position: relative; border-radius: 4px; }
        .panel-title { font-size: 0.7em; opacity: 0.6; margin-bottom: 5px; }
        .val { font-size: 2.2em; font-weight: bold; }
        .unit { font-size: 0.8em; opacity: 0.5; }
        .state-panel { grid-column: 1 / -1; text-align: center; border-width: 2px; }
        .fall-active { background: rgba(255, 0, 0, 0.2); border-color: #ff3333; color: #ff3333; animation: blink 1s infinite; }
        @keyframes blink { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
        .live-tag { position: absolute; top: 5px; right: 10px; font-size: 0.6em; color: #ff3333; animation: pulse 1s infinite; }
        @keyframes pulse { 0% { opacity: 1; } 50% { opacity: 0.3; } 100% { opacity: 1; } }
        .imu-data { font-size: 0.75em; display: grid; grid-template-columns: 1fr 1fr; gap: 5px; margin-top: 10px; }
    </style>
</head>
<body>
    <div class="header">
        <span>ALMA_BRACELET_LINK_v1.0</span>
        <span id="ip-addr">192.168.4.1</span>
    </div>

    <div class="grid">
        <div class="panel state-panel" id="main-status">
            <div class="panel-title">ESTADO_GLOBAL_DEL_SISTEMA</div>
            <div id="l-estado" class="val" style="font-size: 1.5em;">INICIANDO...</div>
        </div>

        <div class="panel">
            <div class="live-tag">● LIVE</div>
            <div class="panel-title">PULSO_CARDIACO</div>
            <div class="val-unit">
                <span id="l-hr" class="val">--</span>
                <span class="unit">BPM</span>
            </div>
        </div>

        <div class="panel">
            <div class="live-tag">● LIVE</div>
            <div class="panel-title">SPO2</div>
            <div class="val-unit">
                <span id="l-spo2" class="val">--</span>
                <span class="unit">%</span>
            </div>
        </div>

        <div class="panel">
            <div class="panel-title">TEMPERATURA</div>
            <div class="val-unit">
                <span id="l-temp" class="val">--</span>
                <span class="unit">°C</span>
            </div>
        </div>

        <div class="panel">
            <div class="panel-title">PRESION</div>
            <div class="val-unit">
                <span id="l-pres" class="val">--</span>
                <span class="unit">hPa</span>
            </div>
        </div>

        <div class="panel">
            <div class="panel-title">ORIENTACION_Y_MOVIMIENTO</div>
            <div class="imu-data">
                <div>ROLL: <span id="l-roll">--</span>°</div>
                <div>PITCH: <span id="l-pitch">--</span>°</div>
                <div>YAW: <span id="l-yaw">--</span>°</div>
                <div>MAG: <span id="l-mag">--</span>G</div>
            </div>
        </div>

        <div class="panel">
            <div class="panel-title">ESTADISTICAS_SISTEMA</div>
            <div class="imu-data">
                <div>CALIDAD_SIG: <span id="l-sig">--</span></div>
                <div>CAIDA: <span id="l-fall">NO</span></div>
            </div>
        </div>
    </div>

    <script>
        const source = new EventSource('/events');
        source.addEventListener('telemetry', function(e) {
            const d = JSON.parse(e.data);
            document.getElementById('l-hr').innerText = d.hr > 0 ? d.hr : "--";
            document.getElementById('l-spo2').innerText = d.spo2 > 0 ? d.spo2 : "--";
            document.getElementById('l-temp').innerText = d.temp.toFixed(1);
            document.getElementById('l-pres').innerText = d.pres.toFixed(1);
            document.getElementById('l-roll').innerText = d.roll.toFixed(0);
            document.getElementById('l-pitch').innerText = d.pitch.toFixed(0);
            document.getElementById('l-yaw').innerText = d.yaw.toFixed(0);
            document.getElementById('l-mag').innerText = d.mag.toFixed(2);
            document.getElementById('l-estado').innerText = d.est;
            document.getElementById('l-sig').innerText = d.sig ? "OK" : "MALA";
            document.getElementById('l-fall').innerText = d.fall ? "!!! SI !!!" : "NO";
            
            const mainStatus = document.getElementById('main-status');
            if(d.fall) mainStatus.classList.add('fall-active');
            else mainStatus.classList.remove('fall-active');
        });
        source.onerror = function(e) { console.error("SSE Error", e); };
    </script>
</body>
</html>
)rawliteral";

// ==================== ESTRUCTURA DE DATOS ====================
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

typedef struct {
  int32_t heartRate;
  int32_t spo2;
  float temperature;
  float pressure;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, yaw;
  float magnitude;
  char estadoGlobal[20];
  bool fallDetected;
  unsigned long fallTimestamp;
  bool signalQuality;
} SensorData;

SensorData g_data;

// =================== MAC Dirección ============================
uint8_t receiverAddress[] = {0x24, 0xDC, 0xC3, 0x48, 0x77, 0x94};
esp_now_peer_info_t peerInfo;

// ==================== CONFIGURACIÓN OLED ======================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C
// ==================== PINES I2C ================================
#define I2C_SDA 21
#define I2C_SCL 22

// ============== CONFIGURACIÓN MAX30102 ==========================
#define MAX30102_SAMPLES 100
#define RATE_SIZE 4
#define SPO2_SIZE 4
#define SIGNAL_QUALITY_THRESHOLD 50000

// ============== Umbrales y Configuración de Caídas ===============
#define FALL_IMPACT_THRESHOLD 3.5
#define FALL_INACTIVITY_TIME 3000
#define FALL_ANGLE_THRESHOLD 50

// ================ Configuración de Temporizadores ================
const unsigned long LORA_INTERVAL = 5000;
const unsigned long OLED_INTERVAL = 300;
const unsigned long WATCHDOG_TIMEOUT = 1000;

// ================ Configuración de Low Power Mode ================
const unsigned long INACTIVITY_TIMEOUT = 300000;
const float MOTION_THRESHOLD = 0.1;
unsigned long lastMotionTime = 0;
bool lowPowerMode = false;

// ================ Instancias de Sensores  ================
MAX30105 particleSensor;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU9250 mpu;
Adafruit_BMP280 bme;

bool bme_present = false;
bool mpu_present = false;
bool max_present = false;
bool oled_present = false;

SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t dataMutex;
TaskHandle_t biometricTaskHandle = NULL;
TaskHandle_t fallTaskHandle = NULL;

// ==================== Buffers y Filtros para MAX30102 ====================
uint32_t irBuffer[MAX30102_SAMPLES];
uint32_t redBuffer[MAX30102_SAMPLES];
uint32_t samplesCollected = 0;
int32_t spo2_buffer[SPO2_SIZE];
int32_t hr_buffer[RATE_SIZE];
uint8_t spo2_buffer_index = 0;
uint8_t hr_buffer_index = 0;

enum BiometricState {
    STATE_NO_FINGER,
    STATE_ACQUIRING,
    STATE_LOCKED
};

struct ConfidenceManager {
    BiometricState state = STATE_NO_FINGER;
    uint32_t lastValidTimestampHR = 0;
    uint32_t lastValidTimestampSpO2 = 0;
    int32_t lastStableHR = 70;
    int32_t lastStableSpO2 = 97;
    int acquiringCycles = 0;
    
    // Parámetros Fisiológicos
    const float MAX_BPM_PER_SEC = 4.0f; 
    const float MAX_SPO2_PER_SEC = 0.5f;
    const int   ACQUIRING_REQUIRED = 5;
    const float MAG_IDEAL = 1.0f;
    const float MAG_TOLERANCE = 0.15f;
    const float MAG_TOLERANCE_SPO2 = 0.25f;
};

ConfidenceManager vld;

// ================ Variables para el Filtro Madgwick ===============
float beta = 0.1f;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float pitchOffset = 0.0f;
float rollOffset = 0.0f;

// ==================== Watchdog por Software ====================
unsigned long lastTaskReset[2] = {0, 0};

// ==================== Prototipos de Funciones ====================
void actualizarOLED();
void enviarTelemetria();
void broadcastTelemetry();
void filtroCaidas(float mag, float inclination);
void aplicarFiltroMadgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt);
void resetI2CBus();
void checkWatchdog();
void inicializarFiltroOrientacion();
void calibrarNivel();
bool sanityCheckHR(int32_t newHR);
bool sanityCheckSpO2(int32_t newSpO2);
void checkLowPowerMode(float magnitude);

// =============================================
// | TAREA CORE 0: BIOMETRÍA                   |
// =============================================
void BiometricTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);

  int32_t spo2, hr;
  int8_t validSPO2, validHR;
  uint32_t signalQualitySum = 0;

  Serial.println("[Tarea0] Biometría optimizada y simplificada iniciada.");

  for (;;) {
    lastTaskReset[0] = millis();

    if (lowPowerMode) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (max_present) {
        particleSensor.check();
        while (particleSensor.available() && samplesCollected < MAX30102_SAMPLES) {
          redBuffer[samplesCollected] = particleSensor.getRed();
          irBuffer[samplesCollected] = particleSensor.getIR();
          particleSensor.nextSample();
          samplesCollected++;
        }
      }
      xSemaphoreGive(i2cMutex);
    }

    if (samplesCollected >= MAX30102_SAMPLES) {
      signalQualitySum = 0;
      for (int i = 0; i < MAX30102_SAMPLES; i++) signalQualitySum += irBuffer[i];
      
      float avgSignal = (float)signalQualitySum / MAX30102_SAMPLES;
      bool goodSignal = (avgSignal > SIGNAL_QUALITY_THRESHOLD);

      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        g_data.signalQuality = goodSignal;
        xSemaphoreGive(dataMutex);
      }

      if (goodSignal) {
        maxim_heart_rate_and_oxygen_saturation(irBuffer, MAX30102_SAMPLES, redBuffer, &spo2, &validSPO2, &hr, &validHR);
        
        if (validHR) sanityCheckHR(hr);
        if (validSPO2) sanityCheckSpO2(spo2);
        
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          if (vld.state >= STATE_ACQUIRING) {
            g_data.heartRate = vld.lastStableHR;
            g_data.spo2 = vld.lastStableSpO2;
          } else {
            g_data.heartRate = 0;
            g_data.spo2 = 0;
          }
          xSemaphoreGive(dataMutex);
        }
      } else {
        Serial.println("[Biometria] Sin señal o mala calidad. Reset.");
        vld.state = STATE_NO_FINGER;
        vld.lastValidTimestampHR = 0;   
        vld.lastValidTimestampSpO2 = 0; 
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          g_data.heartRate = 0;
          g_data.spo2 = 0;
          xSemaphoreGive(dataMutex);
        }
      }

      for (byte i = 25; i < 100; i++) {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
      }
      samplesCollected = 75;
    }

    static unsigned long lastEnvRead = 0;
    if (millis() - lastEnvRead > 1000) {
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (bme_present) {
          float nt = bme.readTemperature();
          float np = bme.readPressure() / 100.0F;
          if (!isnan(nt) && xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            g_data.temperature = nt;
            g_data.pressure = np;
            xSemaphoreGive(dataMutex);
          }
        }
        xSemaphoreGive(i2cMutex);
      }
      lastEnvRead = millis();
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// =============================================
// | SETUP (CORE 1)                            |
// =============================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n\n[ALMA v1000% - WRO 2026] Iniciando...");

  // --- Crear Mutex ---
  i2cMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
    // OLED
    oled_present = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
    if (!oled_present) {
      Serial.println("[ERROR] OLED no detectado. Continuando sin pantalla.");
    } else {
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0, 20);
      display.println("INICIANDO ALMA...");
      display.display();
    }

    // MAX30102
    max_present = particleSensor.begin(Wire, I2C_SPEED_FAST);
    if (!max_present) {
      Serial.println("[ERROR] MAX30102 no detectado.");
    } else {
      particleSensor.setup(0x3F, 1, 2, 100, 411, 4096);
    }

    // BME280
    bme_present = bme.begin(0x76);
    if (!bme_present) {
      Serial.println("[ERROR] BME280 no detectado.");
    }

    // MPU9250
    mpu_present = (mpu.setup(0x68) >= 0);
    if (!mpu_present) {
      Serial.println("[ERROR] MPU9250 no detectado.");
    } else {
      mpu.calibrateAccelGyro();
    }

    xSemaphoreGive(i2cMutex);
  }

  if (mpu_present) {
    inicializarFiltroOrientacion();
    calibrarNivel();
  }

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    strcpy(g_data.estadoGlobal, "INICIANDO");
    g_data.signalQuality = false;
    g_data.heartRate = 0;
    g_data.spo2 = 0;
    xSemaphoreGive(dataMutex);
  }

  lastMotionTime = millis();
  WiFi.mode(WIFI_AP);
  
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  
  if(WiFi.softAP(AP_SSID, AP_PASS)) {
    Serial.println("[WIFI] AP ALMA iniciado. IP: 192.168.4.1");
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.addHandler(&events);
  server.begin();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] Error inicializando ESP-NOW");
    return;
  }

  peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("[ERROR] Fallo al añadir el peer ESP-NOW");
    return;
  }
  
  Serial.println("[Red] WiFi y ESP-NOW listos.");

  xTaskCreatePinnedToCore(
    BiometricTask,
    "BiometricTask",
    10000,
    NULL,
    1,
    &biometricTaskHandle,
    0
  );

  lastTaskReset[0] = millis();
  lastTaskReset[1] = millis();

  Serial.println("[SETUP] Listo.");
}

// =============================================
// | LOOP PRINCIPAL (CORE 1)                   |
// =============================================
void loop() {
  lastTaskReset[1] = millis();
  
  static unsigned long lastMPURead = 0;
  unsigned long currentMicros = micros();
  float dt = 0.01f;

  bool lecturaExitosa = false;

  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {

    if (mpu_present && mpu.update()) {
      float ax = mpu.getAccX();
      float ay = mpu.getAccY();
      float az = mpu.getAccZ();
      float gx = mpu.getGyroX();
      float gy = mpu.getGyroY();
      float gz = mpu.getGyroZ();
      float mx = mpu.getMagX();
      float my = mpu.getMagY();
      float mz = mpu.getMagZ();

      if (currentMicros > lastMPURead) {
        dt = (currentMicros - lastMPURead) / 1000000.0f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.1f) dt = 0.1f;
      }

      aplicarFiltroMadgwick(ax, ay, az, gx, gy, gz, mx, my, mz, dt);

      float asinInput = 2.0f * (q0 * q2 - q3 * q1);
      if (asinInput > 1.0f) asinInput = 1.0f;
      if (asinInput < -1.0f) asinInput = -1.0f;

      float mag = sqrt(ax * ax + ay * ay + az * az);
      float pitch = (asin(asinInput) * 180.0f / PI) - pitchOffset;
      float roll = (atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI) - rollOffset;
      float yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;

      checkLowPowerMode(mag);

      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        g_data.ax = ax;
        g_data.ay = ay;
        g_data.az = az;
        g_data.gx = gx;
        g_data.gy = gy;
        g_data.gz = gz;
        g_data.mx = mx;
        g_data.my = my;
        g_data.mz = mz;
        g_data.magnitude = mag;
        g_data.pitch = pitch;
        g_data.roll = roll;
        g_data.yaw = yaw;
        xSemaphoreGive(dataMutex);
      }
      lecturaExitosa = true;
    }
    xSemaphoreGive(i2cMutex);
  }

  if (lecturaExitosa) {
    lastMPURead = currentMicros;
  }

  SensorData localData;
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(&localData, (const void*)&g_data, sizeof(SensorData));
    xSemaphoreGive(dataMutex);
  }

  if (!lowPowerMode) {
    filtroCaidas(localData.magnitude, localData.pitch);
  }

  static unsigned long lastLoRaSend = 0;
  if (millis() - lastLoRaSend > LORA_INTERVAL && !lowPowerMode) {
    enviarTelemetria();
    lastLoRaSend = millis();
  }

  static unsigned long lastWebSend = 0;
  if (millis() - lastWebSend > 200 && !lowPowerMode) {
    broadcastTelemetry();
    lastWebSend = millis();
  }

  static unsigned long lastOLEDUpdate = 0;
  if (millis() - lastOLEDUpdate > OLED_INTERVAL) {
    if (!lowPowerMode) {
      actualizarOLED();
    } else {
      if (oled_present) {
        display.clearDisplay();
        display.display();
        display.ssd1306_command(SSD1306_DISPLAYOFF);
      }
    }
    lastOLEDUpdate = millis();
  }

  checkWatchdog();

  delay(10);
}

// =============================================
// | IMPLEMENTACIÓN DE FUNCIONES               |
// =============================================

void calibrarNivel() {
  Serial.println("[Calibracion] Estableciendo nivel cero...");
  delay(500);

  float sumPitch = 0;
  float sumRoll = 0;
  int samples = 50;

  for (int i = 0; i < samples; i++) {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      if (mpu.update()) {
        float ax = mpu.getAccX();
        float ay = mpu.getAccY();
        float az = mpu.getAccZ();
        float gx = mpu.getGyroX();
        float gy = mpu.getGyroY();
        float gz = mpu.getGyroZ();
        float mx = mpu.getMagX();
        float my = mpu.getMagY();
        float mz = mpu.getMagZ();

        aplicarFiltroMadgwick(ax, ay, az, gx, gy, gz, mx, my, mz, 0.01f);

        sumPitch += asin(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / PI;
        sumRoll += atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q3)) * 180.0f / PI;
      }
      xSemaphoreGive(i2cMutex);
    }
    delay(10);
  }

  pitchOffset = sumPitch / samples;
  rollOffset = sumRoll / samples;

  Serial.printf("[Calibracion] Offset - Pitch: %.2f, Roll: %.2f\n", pitchOffset, rollOffset);
}
#define HR_FILTER_SIZE 4
int32_t hrHistory[HR_FILTER_SIZE];
uint8_t hrHistoryIdx = 0;

bool sanityCheckHR(int32_t newHR) {
    uint32_t now = millis();
    float currentMag = 1.0f;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        currentMag = g_data.magnitude;
        xSemaphoreGive(dataMutex);
    }
    if (newHR < 45 || newHR > 180) return false;
    float dt = (now - vld.lastValidTimestampHR) / 1000.0f;
    if (dt <= 0 || dt > 2.0f) dt = 0.5f;
    bool motionExcessive = (abs(currentMag - vld.MAG_IDEAL) > vld.MAG_TOLERANCE);

    switch (vld.state) {
        case STATE_NO_FINGER:
            vld.state = STATE_ACQUIRING;
            vld.acquiringCycles = 0;
            vld.lastStableHR = 75; 
            vld.lastValidTimestampHR = now;
            return true; 

        case STATE_ACQUIRING:
            if (abs(newHR - vld.lastStableHR) < (20.0f * dt)) {
                vld.acquiringCycles++;
                vld.lastStableHR = newHR;
                vld.lastValidTimestampHR = now;
                if (vld.acquiringCycles >= vld.ACQUIRING_REQUIRED && !motionExcessive) {
                    vld.state = STATE_LOCKED;
                }
                return true;
            } else {
                if (newHR > vld.lastStableHR) vld.lastStableHR += 5;
                else vld.lastStableHR -= 5;
                return false;
            }

        case STATE_LOCKED:
            if (motionExcessive) return false;
            
            if (newHR > (vld.lastStableHR * 1.8f)) return false; 
            
            float maxAllowed = vld.MAX_BPM_PER_SEC * dt;
            if (abs(newHR - vld.lastStableHR) <= (maxAllowed + 1.5f)) {
                vld.lastStableHR = newHR;
                vld.lastValidTimestampHR = now;
                return true;
            } else {
                static int outlierCount = 0;
                if (++outlierCount > 10) {
                    vld.state = STATE_ACQUIRING;
                    vld.acquiringCycles = 0;
                    outlierCount = 0;
                }
            }
            break;
    }
    return false;
}

bool sanityCheckSpO2(int32_t newSpO2) {
    if (newSpO2 < 85 || newSpO2 > 100) return false;

    float currentMag = 1.0f;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        currentMag = g_data.magnitude;
        xSemaphoreGive(dataMutex);
    }
    if (abs(currentMag - vld.MAG_IDEAL) > vld.MAG_TOLERANCE_SPO2) return false;
    uint32_t now = millis();
    if (vld.lastValidTimestampSpO2 == 0) {
        if (newSpO2 >= 90) {
            vld.lastStableSpO2 = newSpO2;
            vld.lastValidTimestampSpO2 = now;
            return true;
        }
        return false;
    }

    float dt = (now - vld.lastValidTimestampSpO2) / 1000.0f;
    
    if (vld.state == STATE_LOCKED) {
        if (dt > 0 && dt < 2.0f) {
            if (abs(newSpO2 - vld.lastStableSpO2) > (vld.MAX_SPO2_PER_SEC * dt + 1.0f)) {
                return false;
            }
        }
    }

    vld.lastStableSpO2 = newSpO2;
    vld.lastValidTimestampSpO2 = now;
    return true;
}

void checkLowPowerMode(float magnitude) {
  static unsigned long lastWakeTime = 0;

  if (magnitude < (1.0f + MOTION_THRESHOLD) && magnitude > (1.0f - MOTION_THRESHOLD)) {
    if (millis() - lastMotionTime > INACTIVITY_TIMEOUT && !lowPowerMode) {
      lowPowerMode = true;
      Serial.println("[LowPower] Modo ahorro activado - Sin movimiento detectado");

      if (oled_present) {
        display.ssd1306_command(SSD1306_DISPLAYOFF);
      }
    }
  } else {
    lastMotionTime = millis();

    if (lowPowerMode) {
      lowPowerMode = false;
      Serial.println("[LowPower] Despertando por movimiento");

      if (oled_present) {
        display.ssd1306_command(SSD1306_DISPLAYON);
      }
    }
  }
}

void inicializarFiltroOrientacion() {
  Serial.println("[Filtro] Inicializando orientacion...");

  q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;

  for (int i = 0; i < 100; i++) {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      if (mpu.update()) {
        float ax = mpu.getAccX();
        float ay = mpu.getAccY();
        float az = mpu.getAccZ();
        float gx = mpu.getGyroX();
        float gy = mpu.getGyroY();
        float gz = mpu.getGyroZ();
        float mx = mpu.getMagX();
        float my = mpu.getMagY();
        float mz = mpu.getMagZ();

        aplicarFiltroMadgwick(ax, ay, az, gx, gy, gz, mx, my, mz, 0.01f);
      }
      xSemaphoreGive(i2cMutex);
    }
    delay(10);
  }

  Serial.println("[Filtro] Orientacion inicial estabilizada.");
}

void aplicarFiltroMadgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt) {
  if (isnan(ax) || isnan(ay) || isnan(az) || isnan(gx) || isnan(gy) || isnan(gz)) return;
  float magNorm = sqrt(mx * mx + my * my + mz * mz);
  if (magNorm < 0.001f) {
    mx = 0; my = 0; mz = 0;
  }
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
  float accNorm = sqrt(ax * ax + ay * ay + az * az);
  if (accNorm > 0.0f) {
    recipNorm = 1.0f / accNorm;
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
    float magNorm = sqrt(mx * mx + my * my + mz * mz);
    if (magNorm > 0.0f) {
      recipNorm = 1.0f / magNorm;
      mx *= recipNorm; my *= recipNorm; mz *= recipNorm;
    }
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (0.5f - q1q1 - q2q2) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (0.5f - q1q1 - q2q2) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (0.5f - q1q1 - q2q2) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    float sNorm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (sNorm > 0.0f) {
      recipNorm = 1.0f / sNorm;
      s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }
  }

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  float qNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (isnan(qNorm) || qNorm < 0.0001f) {
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    return;
  }

  recipNorm = 1.0f / qNorm;
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

void filtroCaidas(float mag, float inclination) {
  bool currentFallState = false;
  unsigned long now = millis();

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    currentFallState = g_data.fallDetected;
    xSemaphoreGive(dataMutex);
  }

  if (mag > FALL_IMPACT_THRESHOLD && !currentFallState) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      g_data.fallDetected = true;
      g_data.fallTimestamp = now;
      xSemaphoreGive(dataMutex);
    }
    Serial.println("!!! IMPACTO DETECTADO !!!");
  }

  if (currentFallState) {

    if (now - g_data.fallTimestamp > FALL_INACTIVITY_TIME) {

      if (mag > 0.8 && mag < 1.2) {

        if (abs(inclination) > FALL_ANGLE_THRESHOLD) {

          if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            strcpy(g_data.estadoGlobal, "CAIDA CONFIRMADA");
            xSemaphoreGive(dataMutex);
          }
        } else {
          if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            strcpy(g_data.estadoGlobal, "RECUPERADO");
            g_data.fallDetected = false;
            xSemaphoreGive(dataMutex);
          }
        }

      } else {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          strcpy(g_data.estadoGlobal, "MOVIMIENTO");
          g_data.fallDetected = false;
          xSemaphoreGive(dataMutex);
        }
      }

    } else {
      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        strcpy(g_data.estadoGlobal, "ANALIZANDO...");
        xSemaphoreGive(dataMutex);
      }

    }
  } else {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      if (mag > 2.0 && mag < 3.5)
        strcpy(g_data.estadoGlobal, "CAMINANDO");
      else if (mag > 3.5)
        strcpy(g_data.estadoGlobal, "CORRIENDO");
      else if (abs(inclination) > 70)
        strcpy(g_data.estadoGlobal, "ACOSTADO");
      else
        strcpy(g_data.estadoGlobal, "ESTABLE");
      xSemaphoreGive(dataMutex);
    }
  }
}

void actualizarOLED() {
  if (!oled_present) return;

  SensorData localData;
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(&localData, (const void*)&g_data, sizeof(SensorData));
    xSemaphoreGive(dataMutex);
  }

  char buffer[20];
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 12);
  sprintf(buffer, "BPM:%3ld SpO2:%3ld%%", localData.heartRate, localData.spo2);
  display.print(buffer);
  if (localData.signalQuality) {
    display.fillRect(100, 14, 20, 6, WHITE);
  } else {
    display.drawRect(100, 14, 20, 6, WHITE);
  }
  display.setCursor(0, 24);
  sprintf(buffer, "T:%.2f", localData.temperature);
  display.print(buffer);
  display.setCursor(0, 36);
  sprintf(buffer, "P:%.1fhPa", localData.pressure);
  display.print(buffer);
  display.setCursor(0, 48);
  display.print("EST:");
  display.setCursor(30, 48);
  display.print(localData.estadoGlobal);

  display.display();
}

void enviarTelemetria() {
  SensorData localData;
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(&localData, (const void*)&g_data, sizeof(SensorData));
    xSemaphoreGive(dataMutex);
  }

  TelemetryPacket packet;
  packet.timestamp = millis();
  packet.heartRate = localData.heartRate;
  packet.spo2 = localData.spo2;
  packet.temperature = localData.temperature;
  packet.pressure = localData.pressure;
  packet.roll = localData.roll;
  packet.pitch = localData.pitch;
  packet.yaw = localData.yaw;
  packet.magnitude = localData.magnitude;
  
  strncpy(packet.estadoGlobal, localData.estadoGlobal, sizeof(packet.estadoGlobal) - 1);
  packet.estadoGlobal[sizeof(packet.estadoGlobal) - 1] = '\0';
  packet.fallDetected = localData.fallDetected ? 1 : 0;
  packet.signalQuality = localData.signalQuality ? 1 : 0;

  if ((packet.heartRate > 0) && (packet.spo2 > 0)) {
    
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &packet, sizeof(packet));
     
    if (result == ESP_OK) {
      Serial.printf("[ESP-NOW] Envío OK | HR: %ld | Est: %s\n", packet.heartRate, packet.estadoGlobal);
    } else {
      Serial.printf("[ERROR] Código: %d | MAC: ", result);
      for(int i=0; i<6; i++) Serial.printf("%02X%s", receiverAddress[i], (i<5)?":":"");
      Serial.println();
    }
  }
}

void resetI2CBus() {
  Serial.println("[WATCHDOG] Reiniciando bus I2C...");
  Wire.end();
  delay(100);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
}

void checkWatchdog() {
  unsigned long now = millis();

  if (now - lastTaskReset[0] > WATCHDOG_TIMEOUT) {
    Serial.println("[WATCHDOG] Timeout en BiometricTask! Reiniciando...");
    if (biometricTaskHandle != NULL) {
      vTaskDelete(biometricTaskHandle);
      xTaskCreatePinnedToCore(BiometricTask, "BiometricTask", 10000, NULL, 1, &biometricTaskHandle, 0);
    }
    resetI2CBus();
    lastTaskReset[0] = millis();
  }

  if (now - lastTaskReset[1] > WATCHDOG_TIMEOUT * 2) {
    Serial.println("[WATCHDOG] Timeout en Loop Principal! Reseteando I2C...");
    resetI2CBus();
    lastTaskReset[1] = millis();
    }
    }

    void broadcastTelemetry() {
    SensorData localData;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(&localData, (const void*)&g_data, sizeof(SensorData));
    xSemaphoreGive(dataMutex);
    }

    StaticJsonDocument<512> doc;
    doc["hr"] = localData.heartRate;
    doc["spo2"] = localData.spo2;
    doc["temp"] = localData.temperature;
    doc["pres"] = localData.pressure;
    doc["roll"] = localData.roll;
    doc["pitch"] = localData.pitch;
    doc["yaw"] = localData.yaw;
    doc["mag"] = localData.magnitude;
    doc["est"] = localData.estadoGlobal;
    doc["fall"] = localData.fallDetected ? 1 : 0;
    doc["sig"] = localData.signalQuality ? 1 : 0;

    String jsonResponse;
    serializeJson(doc, jsonResponse);
    events.send(jsonResponse.c_str(), "telemetry", millis());
    }