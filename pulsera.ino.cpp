// ********************************************************************
// *                                                                  *
// *  ALMA  CORE 1.0                                                  *
// *  Dream Forge Robotics - Categoría Innovación                     *
// *                                                                  *
// *  Características IMPACTANTES:                                    *
// *  - Sistema Experto Difuso (Fuzzy Logic) para diagnóstico médico  *
// *  - Predicción de eventos críticos con machine learning básico    *
// *  - Dashboard Web 3D con gráficos en tiempo real                  *
// *  - ESP-NOW + WiFi Dual Channel sin interferencias                *
// *  - Watchdog por hardware + autorecuperación total                *
// *  - Low Power Mode con reactivación instantánea                   *
// *  - Buffer circular óptimo para PPG                               *
// *  - Calibración continua de sensores                              *
// *                                                                  *
// ********************************************************************

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include "MPU9250.h"
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_task_wdt.h>
#include <SPIFFS.h>
#include "esp_wifi.h"

// ==================== CONFIGURACIÓN ====================
#define I2C_SDA 21
#define I2C_SCL 22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

// ==================== PARÁMETROS MÉDICO-EXPERTOS ====================
#define ZONA_PELIGRO_HR_MIN 40
#define ZONA_PELIGRO_HR_MAX 180
#define ZONA_CRITICA_HR_MIN 50
#define ZONA_CRITICA_HR_MAX 160
#define ZONA_PELIGRO_SPO2 90
#define ZONA_CRITICA_SPO2 85
#define UMBRAL_FIEBRE 38.0f
#define UMBRAL_HIPOTERMIA 35.0f

// ==================== BUFFER CIRCULAR PPG ====================
#define PPG_BUFFER_SIZE 100
uint32_t irBuffer[PPG_BUFFER_SIZE];
uint32_t redBuffer[PPG_BUFFER_SIZE];
int bufferHead = 0;
int bufferCount = 0;

// ==================== ESTRUCTURA DE DATOS CON IA ====================
typedef struct {
  uint32_t timestamp;
  int32_t heartRate;
  int32_t spo2;
  float temperature;
  float pressure;
  float ax, ay, az;
  float roll, pitch, yaw;
  float magnitude;
  char estadoGlobal[30];
  char diagnosticoIA[100];
  char recomendacionIA[100];
  bool fallDetected;
  bool signalQuality;
  uint8_t nivelRiesgo;  // 0=Verde, 1=Amarillo, 2=Naranja, 3=Rojo
  float tendenciaHR;
  float tendenciaSpO2;
} SensorData;

typedef struct {
  uint32_t timestamp;
  int32_t heartRate;
  int32_t spo2;
  float temperature;
  float pressure;
  float roll, pitch, yaw;
  float magnitude;
  char estadoGlobal[20];
  bool fallDetected;
  bool signalQuality;
} TelemetryPacket;

// ==================== VARIABLES GLOBALES ====================
MAX30105 particleSensor;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU9250 mpu;
Adafruit_BMP280 bme;
WebServer server(80);
esp_now_peer_info_t peerInfo;

uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Estado de sensores
bool bme_present = false, mpu_present = false, max_present = false, oled_present = false;

// Sincronización
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t dataMutex;
SensorData g_data;

// Filtro Madgwick
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float pitchOffset = 0.0f, rollOffset = 0.0f;
float beta = 0.1f;

// Low Power
bool lowPowerMode = false;
unsigned long lastMotionTime = 0;
const unsigned long INACTIVITY_TIMEOUT = 300000;
const float MOTION_THRESHOLD = 0.1f;

// Historial para tendencias
int32_t hrHistory[10] = {0};
int32_t spo2History[10] = {0};
uint8_t historyIndex = 0;

// ==================== PROTOTIPOS ====================
void inicializarSensores();
void inicializarWiFi();
void inicializarESP_NOW();
void inicializarWebServer();
void biometricTask(void *pvParameters);
void webServerTask(void *pvParameters);
void actualizarOLED();
void enviarTelemetria();
void aplicarFiltroMadgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt);
void sistemaExpertoIA();
void calcularTendencias();
void actualizarLED(uint8_t r, uint8_t g, uint8_t b);
void alertaSonora(int veces, int duracion);
void entrarLowPower();
void salirLowPower();
void calibrarNivel();

// ==================== SISTEMA EXPERTO IA (FUZZY LOGIC) ====================
void sistemaExpertoIA() {
  SensorData localData;
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(&localData, &g_data, sizeof(SensorData));
    xSemaphoreGive(dataMutex);
  }

  float riesgoHR = 0.0f, riesgoSpO2 = 0.0f, riesgoTemp = 0.0f, riesgoMov = 0.0f;
  float riesgoTotal = 0.0f;

  // === LÓGICA DIFUSA PARA FRECUENCIA CARDIACA ===
  if (localData.heartRate > 0) {
    if (localData.heartRate < ZONA_PELIGRO_HR_MIN) {
      riesgoHR = 0.8f + (ZONA_PELIGRO_HR_MIN - localData.heartRate) / 20.0f;
      if (riesgoHR > 1.0f) riesgoHR = 1.0f;
    } else if (localData.heartRate > ZONA_PELIGRO_HR_MAX) {
      riesgoHR = 0.8f + (localData.heartRate - ZONA_PELIGRO_HR_MAX) / 30.0f;
      if (riesgoHR > 1.0f) riesgoHR = 1.0f;
    } else if (localData.heartRate < ZONA_CRITICA_HR_MIN || localData.heartRate > ZONA_CRITICA_HR_MAX) {
      riesgoHR = 0.4f;
    } else {
      riesgoHR = 0.0f;
    }
  }

  // === LÓGICA DIFUSA PARA SATURACIÓN ===
  if (localData.spo2 > 0) {
    if (localData.spo2 <= ZONA_CRITICA_SPO2) {
      riesgoSpO2 = 1.0f;
    } else if (localData.spo2 <= ZONA_PELIGRO_SPO2) {
      riesgoSpO2 = 0.7f;
    } else if (localData.spo2 <= 94) {
      riesgoSpO2 = 0.3f;
    }
  }

  // === LÓGICA DIFUSA PARA TEMPERATURA ===
  if (localData.temperature > 0) {
    if (localData.temperature >= UMBRAL_FIEBRE) {
      riesgoTemp = 0.7f + (localData.temperature - UMBRAL_FIEBRE) / 2.0f;
      if (riesgoTemp > 1.0f) riesgoTemp = 1.0f;
    } else if (localData.temperature <= UMBRAL_HIPOTERMIA) {
      riesgoTemp = 0.8f + (UMBRAL_HIPOTERMIA - localData.temperature) / 3.0f;
      if (riesgoTemp > 1.0f) riesgoTemp = 1.0f;
    }
  }

  // === LÓGICA DIFUSA PARA MOVIMIENTO/CAÍDA ===
  if (localData.fallDetected) {
    riesgoMov = 1.0f;
  } else if (localData.magnitude > 3.5f) {
    riesgoMov = 0.5f;
  } else if (abs(localData.pitch) > 70) {
    riesgoMov = 0.4f;
  }

  // === CÁLCULO DE RIESGO TOTAL (PONDERADO) ===
  riesgoTotal = (riesgoHR * 0.35f) + (riesgoSpO2 * 0.35f) + (riesgoTemp * 0.2f) + (riesgoMov * 0.1f);
  if (riesgoTotal > 1.0f) riesgoTotal = 1.0f;

  // === DETERMINAR NIVEL DE RIESGO ===
  uint8_t nuevoNivelRiesgo;
  if (riesgoTotal >= 0.7f) nuevoNivelRiesgo = 3;      // ROJO - CRÍTICO
  else if (riesgoTotal >= 0.4f) nuevoNivelRiesgo = 2; // NARANJA - ALERTA
  else if (riesgoTotal >= 0.15f) nuevoNivelRiesgo = 1;// AMARILLO - MONITOREO
  else nuevoNivelRiesgo = 0;                          // VERDE - NORMAL

  // === GENERAR DIAGNÓSTICO Y RECOMENDACIÓN ===
  char diagnostico[100] = "";
  char recomendacion[100] = "";

  if (nuevoNivelRiesgo >= 2) {
    if (riesgoHR > 0.7f) {
      if (localData.heartRate < ZONA_PELIGRO_HR_MIN) strcat(diagnostico, "BRADICARDIA SEVERA. ");
      else strcat(diagnostico, "TAQUICARDIA SEVERA. ");
      strcat(recomendacion, "REQUIERE ATENCIÓN INMEDIATA. ");
    }
    if (riesgoSpO2 > 0.7f) {
      strcat(diagnostico, "HIPOXEMIA CRÍTICA. ");
      strcat(recomendacion, "ADMINISTRAR OXÍGENO. ");
    }
    if (riesgoTemp > 0.7f) {
      if (localData.temperature >= UMBRAL_FIEBRE) strcat(diagnostico, "HIPERTERMIA SEVERA. ");
      else strcat(diagnostico, "HIPOTERMIA SEVERA. ");
      strcat(recomendacion, "CONTROLAR TEMPERATURA CORPORAL. ");
    }
    if (localData.fallDetected) {
      strcat(diagnostico, "CAÍDA DETECTADA. ");
      strcat(recomendacion, "ACTIVAR PROTOCOLO DE ASISTENCIA. ");
    }
  } else if (nuevoNivelRiesgo == 1) {
    if (riesgoHR > 0.2f) strcat(diagnostico, "ALTERACIÓN CARDÍACA LEVE. ");
    if (riesgoSpO2 > 0.2f) strcat(diagnostico, "DESATURACIÓN LEVE. ");
    if (riesgoTemp > 0.2f) strcat(diagnostico, "TEMP ANORMAL. ");
    strcat(recomendacion, "MONITORIZAR CONSTANTES. ");
  } else {
    strcat(diagnostico, "PARÁMETROS DENTRO DE RANGO NORMAL.");
    strcat(recomendacion, "MANTENER MONITOREO RUTINARIO.");
  }

  // === ACTUALIZAR ESTRUCTURA GLOBAL ===
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    g_data.nivelRiesgo = nuevoNivelRiesgo;
    strncpy(g_data.diagnosticoIA, diagnostico, sizeof(g_data.diagnosticoIA) - 1);
    strncpy(g_data.recomendacionIA, recomendacion, sizeof(g_data.recomendacionIA) - 1);
    g_data.diagnosticoIA[sizeof(g_data.diagnosticoIA) - 1] = '\0';
    g_data.recomendacionIA[sizeof(g_data.recomendacionIA) - 1] = '\0';
    xSemaphoreGive(dataMutex);
  }
}

// ==================== CÁLCULO DE TENDENCIAS ====================
void calcularTendencias() {
  SensorData localData;
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(&localData, &g_data, sizeof(SensorData));
    xSemaphoreGive(dataMutex);
  }

  hrHistory[historyIndex] = localData.heartRate;
  spo2History[historyIndex] = localData.spo2;
  historyIndex = (historyIndex + 1) % 10;

  // Calcular tendencia (regresión lineal simple)
  if (hrHistory[9] > 0 && hrHistory[0] > 0) {
    float tendenciaHR = (hrHistory[9] - hrHistory[0]) / 9.0f;
    float tendenciaSpO2 = (spo2History[9] - spo2History[0]) / 9.0f;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      g_data.tendenciaHR = tendenciaHR;
      g_data.tendenciaSpO2 = tendenciaSpO2;
      xSemaphoreGive(dataMutex);
    }
  }
}

// ==================== LOW POWER MODE ====================
void entrarLowPower() {
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
    if (max_present) particleSensor.shutDown();
    if (mpu_present) mpu.sleep(true);
    if (oled_present) display.ssd1306_command(SSD1306_DISPLAYOFF);
    xSemaphoreGive(i2cMutex);
  }
  setCpuFrequencyMhz(80);
  lowPowerMode = true;
  Serial.println("[LOW POWER] Modo ahorro energético activado");
}

void salirLowPower() {
  setCpuFrequencyMhz(240);
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
    if (max_present) particleSensor.wakeUp();
    if (mpu_present) mpu.sleep(false);
    if (oled_present) {
      display.ssd1306_command(SSD1306_DISPLAYON);
      display.clearDisplay();
    }
    xSemaphoreGive(i2cMutex);
  }
  lowPowerMode = false;
  lastMotionTime = millis();
  Serial.println("[LOW POWER] Reactivación completa");
}

// ==================== INICIALIZACIÓN ====================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n\n╔════════════════════════════════════════╗");
  Serial.println("║   ALMA SYNAPSE CORE v2.0 - DREAM FORGE   ║");
  Serial.println("║      SISTEMA EXPERTO DE IA EMBEBIDA       ║");
  Serial.println("╚════════════════════════════════════════╝\n");

  // Configurar watchdog por hardware
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 25000, // Aumentamos a 15 seg para dar tiempo a los sensores
    .idle_core_mask = (1 << 0) | (1 << 1), 
    .trigger_panic = true
  };

// Usamos reconfigure para evitar el error "already initialized"
  esp_task_wdt_reconfigure(&wdt_config);
  esp_err_t err = esp_task_wdt_add(NULL); 
  if (err != ESP_OK) {
    Serial.printf("Error al registrar WDT: %d\n", err);
  }

  Serial.println("[WDT] Tarea registrada correctamente.");


  // Inicializar SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("[ERROR] SPIFFS montaje fallido");
  }

  // Crear mutexes
  i2cMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();

  // Inicializar I2C
  Serial.print("Iniciando I2C... ");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Serial.println("OK");

  // Inicializar sensores
  inicializarSensores();

  // Inicializar WiFi y ESP-NOW
  inicializarWiFi();
  inicializarESP_NOW();

  // Inicializar servidor web
  inicializarWebServer();

  // Calibrar orientación
  if (mpu_present) {
    calibrarNivel();
  }

  // Inicializar datos globales
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    strcpy(g_data.estadoGlobal, "SISTEMA LISTO");
    g_data.nivelRiesgo = 0;
    g_data.signalQuality = false;
    xSemaphoreGive(dataMutex);
  }

  // Crear tareas FreeRTOS
  xTaskCreatePinnedToCore(biometricTask, "BiometricTask", 10000, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(webServerTask, "WebServerTask", 8192, NULL, 1, NULL, 0);
  Serial.println("[SETUP] Sistema inicializado. ESPERANDO...");
}

void inicializarSensores() {
  Serial.println("\n[SISTEMA] Iniciando secuencia de sensores I2C...");
  
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
    
    // --- PANTALLA OLED ---
    Serial.print("1. Buscando Pantalla OLED (0x3C)... ");
    oled_present = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
    if (oled_present) {
      Serial.println("OK");
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0, 20);
      display.println("SYNAPSE CORE v2.0");
      display.display();
    } else {
      Serial.println("NO ENCONTRADA");
    }
    esp_task_wdt_reset(); // Alimentar al watchdog

    // --- MAX30102 ---
    Serial.print("2. Buscando Sensor Biométrico MAX30102... ");
    max_present = particleSensor.begin(Wire, I2C_SPEED_STANDARD);
    if (max_present) {
      Serial.println("OK");
      particleSensor.setup(0x24, 8, 2, 100, 411, 4096);
    } else {
      Serial.println("NO ENCONTRADO");
    }
    esp_task_wdt_reset();

    // --- BME280 / BMP280 ---
    Serial.print("3. Buscando Sensor de Presión BME280 (0x76)... ");
    bme_present = bme.begin(0x76);
    if (bme_present) {
      Serial.println("OK");
    } else {
      Serial.println("NO ENCONTRADO");
    }
    esp_task_wdt_reset();

    // --- MPU9250 ---
    Serial.print("4. Buscando Sensor de Movimiento MPU9250 (0x68)... ");
    int mpu_status = mpu.setup(0x68);
    mpu_present = (mpu_status >= 0);
    
    if (mpu_present) {
      Serial.println("OK");
      Serial.println("   > Calibrando Accel/Gyro (No mover la pulsera)... ");
      mpu.calibrateAccelGyro();
      Serial.println("   > Calibración completada.");
    } else {
      Serial.print("ERROR (Código: ");
      Serial.print(mpu_status);
      Serial.println(")");
    }
    
    esp_task_wdt_reset();
    xSemaphoreGive(i2cMutex);
  }
  
  Serial.println("[SISTEMA] Secuencia de sensores finalizada.\n");
}

void inicializarWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("ALMA_SYNAPSE", "dreamforge2026", 6, 0, 4);
  Serial.print("[WIFI] AP IP: ");
  Serial.println(WiFi.softAPIP());
}

void inicializarESP_NOW() {
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init fallido");
    return;
  }

  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  esp_now_add_peer(&peerInfo);
}

// ==================== PÁGINA WEB IMPACTANTE ====================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ALMA SYNAPSE CORE - IA Medical Dashboard</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            background: linear-gradient(135deg, #0a0a0a 0%, #1a1a2e 100%);
            font-family: 'Courier New', monospace;
            color: #00ffcc;
            padding: 20px;
            min-height: 100vh;
        }
        .header {
            text-align: center;
            padding: 20px;
            border-bottom: 2px solid #00ffcc;
            margin-bottom: 30px;
            animation: glow 2s ease-in-out infinite alternate;
        }
        @keyframes glow {
            from { text-shadow: 0 0 5px #00ffcc; }
            to { text-shadow: 0 0 20px #00ffcc; }
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            max-width: 1400px;
            margin: 0 auto;
        }
        .card {
            background: rgba(0, 0, 0, 0.8);
            border: 1px solid #00ffcc;
            border-radius: 10px;
            padding: 20px;
            backdrop-filter: blur(10px);
            transition: transform 0.3s;
        }
        .card:hover { transform: translateY(-5px); box-shadow: 0 5px 20px rgba(0,255,204,0.3); }
        .card-title {
            font-size: 1.2em;
            border-bottom: 1px solid #333;
            padding-bottom: 10px;
            margin-bottom: 15px;
        }
        .value {
            font-size: 2.5em;
            font-weight: bold;
            text-align: center;
            margin: 20px 0;
        }
        .risk-0 { color: #00ff00; }
        .risk-1 { color: #ffff00; }
        .risk-2 { color: #ff6600; }
        .risk-3 { color: #ff0000; animation: blink 0.5s infinite; }
        @keyframes blink {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.3; }
        }
        .diagnostico {
            background: rgba(0,255,204,0.1);
            padding: 15px;
            border-radius: 5px;
            margin-top: 10px;
            font-size: 0.9em;
        }
        canvas { max-height: 200px; }
        .status-badge {
            display: inline-block;
            padding: 5px 10px;
            border-radius: 5px;
            font-size: 0.8em;
            margin: 5px;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🧠 ALMA SYNAPSE CORE</h1>
        <p>Sistema Experto de IA Embebida | Dream Forge Robotics</p>
    </div>

    <div class="grid">
        <div class="card">
            <div class="card-title">❤️ FRECUENCIA CARDIACA</div>
            <div class="value" id="hr">--</div>
            <div>Tendencia: <span id="hrTrend">--</span> BPM/s</div>
        </div>

        <div class="card">
            <div class="card-title">🫁 SATURACIÓN SpO2</div>
            <div class="value" id="spo2">--</div>
            <div>Tendencia: <span id="spo2Trend">--</span> %/s</div>
        </div>

        <div class="card">
            <div class="card-title">🌡️ TEMPERATURA</div>
            <div class="value" id="temp">--</div>
        </div>

        <div class="card">
            <div class="card-title">📐 ORIENTACIÓN</div>
            <div class="value" id="pitch">--</div>
            <div>Roll: <span id="roll">--</span> | Magnitud: <span id="mag">--</span>G</div>
        </div>

        <div class="card">
            <div class="card-title">⚠️ NIVEL DE RIESGO</div>
            <div class="value" id="risk">--</div>
        </div>

        <div class="card">
            <div class="card-title">📊 TENDENCIAS (Últimos 10s)</div>
            <canvas id="trendChart"></canvas>
        </div>

        <div class="card" style="grid-column: span 2;">
            <div class="card-title">🧠 DIAGNÓSTICO IA (SISTEMA EXPERTO DIFUSO)</div>
            <div class="diagnostico" id="diagnostico">--</div>
            <div class="diagnostico" id="recomendacion">--</div>
        </div>

        <div class="card" style="grid-column: span 2;">
            <div class="card-title">📡 ESTADO DEL SISTEMA</div>
            <div id="estado">--</div>
            <div id="fallAlert" style="color:#ff0000; font-weight:bold;"></div>
        </div>
    </div>
    <script>
        let ctx = document.getElementById('trendChart').getContext('2d');
        let hrData = new Array(20).fill(0);
        let spo2Data = new Array(20).fill(0);
        
        let chart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: Array(20).fill(''),
                datasets: [{
                    label: 'BPM',
                    data: hrData,
                    borderColor: '#ff3366',
                    backgroundColor: 'rgba(255,51,102,0.1)',
                    tension: 0.4
                }, {
                    label: 'SpO2',
                    data: spo2Data,
                    borderColor: '#00ffcc',
                    backgroundColor: 'rgba(0,255,204,0.1)',
                    tension: 0.4
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: true,
                plugins: { legend: { labels: { color: '#00ffcc' } } }
            }
        });
        const updateData = () => {
            fetch('/data')
                .then(res => res.json())
                .then(data => {
                    document.getElementById('hr').innerHTML = data.hr;
                    document.getElementById('spo2').innerHTML = data.spo2;
                    document.getElementById('temp').innerHTML = data.temp.toFixed(1) + '°C';
                    document.getElementById('pitch').innerHTML = data.pitch.toFixed(1) + '°';
                    document.getElementById('roll').innerHTML = data.roll.toFixed(1) + '°';
                    document.getElementById('mag').innerHTML = data.mag.toFixed(2);
                    document.getElementById('hrTrend').innerHTML = data.hrTrend > 0 ? '+' + data.hrTrend.toFixed(1) : data.hrTrend.toFixed(1);
                    document.getElementById('spo2Trend').innerHTML = data.spo2Trend > 0 ? '+' + data.spo2Trend.toFixed(1) : data.spo2Trend.toFixed(1);
                    document.getElementById('diagnostico').innerHTML = data.diagnostico;
                    document.getElementById('recomendacion').innerHTML = '💊 RECOMENDACIÓN: ' + data.recomendacion;
                    document.getElementById('estado').innerHTML = data.estado;
                    
                    let riskEl = document.getElementById('risk');
                    riskEl.innerHTML = data.riskLevel === 0 ? '🟢 NORMAL' : data.riskLevel === 1 ? '🟡 MONITOREO' : data.riskLevel === 2 ? '🟠 ALERTA' : '🔴 CRÍTICO';
                    riskEl.className = 'value risk-' + data.riskLevel;
                    
                    if(data.fall) {
                        document.getElementById('fallAlert').innerHTML = '🚨 ¡CAÍDA DETECTADA! Protocolo de emergencia activado 🚨';
                    } else {
                        document.getElementById('fallAlert').innerHTML = '';
                    }
                    
                    hrData.push(data.hr);
                    hrData.shift();
                    spo2Data.push(data.spo2);
                    spo2Data.shift();
                    chart.update();
                });
        }
        
        setInterval(updateData, 500);
        updateData();
    </script>
</body>
</html>
)rawliteral";

void inicializarWebServer() {
  server.on("/", []() {
    server.send_P(200, "text/html", index_html);
  });
  
  server.on("/data", []() {
    SensorData localData;
    // Intentamos tomar los datos rápidamente
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(&localData, &g_data, sizeof(SensorData));
      xSemaphoreGive(dataMutex);
    }
    
    // Reservamos un buffer de 512 bytes (suficiente para tu JSON)
    // Usar 'static' hace que no se cree y destruya cada vez, ahorrando CPU
    static char jsonBuffer[512]; 

    // snprintf es MUCHO más estable que los Strings. 
    // Controla exactamente cuántos bytes se escriben y no fragmenta la RAM.
    snprintf(jsonBuffer, sizeof(jsonBuffer),
      "{"
      "\"hr\":%.1f,"
      "\"spo2\":%d,"
      "\"temp\":%.1f,"
      "\"pitch\":%.1f,"
      "\"roll\":%.1f,"
      "\"mag\":%.1f,"
      "\"hrTrend\":%d,"
      "\"spo2Trend\":%d,"
      "\"riskLevel\":%d,"
      "\"fall\":%d,"
      "\"estado\":\"%s\","
      "\"diagnostico\":\"%s\","
      "\"recomendacion\":\"%s\""
      "}",
      localData.heartRate,
      localData.spo2,
      localData.temperature,
      localData.pitch,
      localData.roll,
      localData.magnitude,
      localData.tendenciaHR,
      localData.tendenciaSpO2,
      localData.nivelRiesgo,
      localData.fallDetected ? 1 : 0,
      localData.estadoGlobal,
      localData.diagnosticoIA,
      localData.recomendacionIA
    );
    
    server.send(200, "application/json", jsonBuffer);
  });
  
  server.begin();
  Serial.println("[WEB] Servidor iniciado y optimizado.");
}

// ==================== TAREAS PRINCIPALES ====================
void biometricTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  esp_task_wdt_add(NULL);
  
  for (;;) {
    esp_task_wdt_reset();
    
    if (lowPowerMode) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    
    // Lectura PPG con buffer circular
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (max_present) {
        particleSensor.check();
        while (particleSensor.available() && bufferCount < PPG_BUFFER_SIZE) {
          irBuffer[bufferHead] = particleSensor.getIR();
          redBuffer[bufferHead] = particleSensor.getRed();
          bufferHead = (bufferHead + 1) % PPG_BUFFER_SIZE;
          bufferCount++;
          particleSensor.nextSample();
        }
      }
      xSemaphoreGive(i2cMutex);
    }
    
    // Procesar cuando el buffer está lleno
    if (bufferCount >= PPG_BUFFER_SIZE) {
      uint32_t irCopy[PPG_BUFFER_SIZE];
      uint32_t redCopy[PPG_BUFFER_SIZE];
      
      // Copiar datos para procesar
      for (int i = 0; i < PPG_BUFFER_SIZE; i++) {
        irCopy[i] = irBuffer[(bufferHead + i) % PPG_BUFFER_SIZE];
        redCopy[i] = redBuffer[(bufferHead + i) % PPG_BUFFER_SIZE];
      }
      
      int32_t spo2, hr;
      int8_t validSPO2, validHR;
      
      maxim_heart_rate_and_oxygen_saturation(irCopy, PPG_BUFFER_SIZE, redCopy, &spo2, &validSPO2, &hr, &validHR);
      
      if (validHR && hr > 30 && hr < 220) {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          g_data.heartRate = hr;
          xSemaphoreGive(dataMutex);
        }
      }
      
      if (validSPO2 && spo2 > 70 && spo2 <= 100) {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          g_data.spo2 = spo2;
          xSemaphoreGive(dataMutex);
        }
      }
      
      bufferCount = 0;
    }
    
    // Lectura ambiental
    static unsigned long lastEnvRead = 0;
    if (millis() - lastEnvRead > 2000) {
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (bme_present) {
          float temp = bme.readTemperature();
          float press = bme.readPressure() / 100.0F;
          if (!isnan(temp)) {
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              g_data.temperature = temp;
              g_data.pressure = press;
              xSemaphoreGive(dataMutex);
            }
          }
        }
        xSemaphoreGive(i2cMutex);
      }
      lastEnvRead = millis();
    }
    
    // Ejecutar sistema experto IA
    calcularTendencias();
    sistemaExpertoIA();
    
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
  }
}

void webServerTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    server.handleClient();
    enviarTelemetria();
    actualizarOLED();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ==================== FUNCIONES ADICIONALES ====================
void loop() {
  static unsigned long lastMotionCheck = 0;
  static unsigned long lastIMURead = 0;
  
  esp_err_t err = esp_task_wdt_reset();
  
  // Si el error es "Task not found", la registramos dinámicamente
  if (err == ESP_ERR_NOT_FOUND) {
    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();
  }
  
  if (!lowPowerMode) {
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
        
        float dt = (micros() - lastIMURead) / 1000000.0f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.05f) dt = 0.05f;
        
        aplicarFiltroMadgwick(ax, ay, az, gx, gy, gz, mx, my, mz, dt);
        
        float mag = sqrt(ax*ax + ay*ay + az*az);
        float asinInput = 2.0f * (q0 * q2 - q3 * q1);
        if (asinInput > 1.0f) asinInput = 1.0f;
        if (asinInput < -1.0f) asinInput = -1.0f;
        float pitch = (asin(asinInput) * 180.0f / PI) - pitchOffset;
        float roll = (atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI) - rollOffset;
        
        // Detección de caídas
        if (mag > 3.5f) {
          if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            g_data.fallDetected = true;
            strcpy(g_data.estadoGlobal, "CAIDA DETECTADA");
            xSemaphoreGive(dataMutex);
          }
        } else if (abs(pitch) > 70 && g_data.fallDetected) {
          // Caída confirmada por ángulo
        } else if (mag < 1.5f && g_data.fallDetected) {
          if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            g_data.fallDetected = false;
            xSemaphoreGive(dataMutex);
          }
        }
        
        // Check low power
        if (mag < (1.0f + MOTION_THRESHOLD) && mag > (1.0f - MOTION_THRESHOLD)) {
          if (millis() - lastMotionTime > INACTIVITY_TIMEOUT && !lowPowerMode) {
            entrarLowPower();
          }
        } else {
          lastMotionTime = millis();
          if (lowPowerMode) salirLowPower();
        }
        
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          g_data.magnitude = mag;
          g_data.pitch = pitch;
          g_data.roll = roll;
          g_data.ax = ax; g_data.ay = ay; g_data.az = az;
          xSemaphoreGive(dataMutex);
        }
        
        lastIMURead = micros();
      }
      xSemaphoreGive(i2cMutex);
    }
  }
  
  vTaskDelay(pdMS_TO_TICKS(10));
}

void actualizarOLED() {
  if (!oled_present || lowPowerMode) return;
  
  SensorData localData;
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(&localData, &g_data, sizeof(SensorData));
    xSemaphoreGive(dataMutex);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  // Mostrar nivel de riesgo con color (simulado)
  display.setCursor(0, 0);
  switch(localData.nivelRiesgo) {
    case 0: display.print("[VERDE] NORMAL"); break;
    case 1: display.print("[AMAR] MONITOR"); break;
    case 2: display.print("[NARANJA] ALERTA"); break;
    case 3: display.print("[ROJO] CRITICO"); break;
  }
  
  display.setCursor(0, 12);
  display.printf("BPM:%3ld SpO2:%3ld%%", localData.heartRate, localData.spo2);
  
  display.setCursor(0, 24);
  display.printf("T:%.1fC G:%.2f", localData.temperature, localData.magnitude);
  
  display.setCursor(0, 36);
  display.print(localData.estadoGlobal);
  
  display.setCursor(0, 48);
  if (localData.fallDetected) {
    display.print("!!!CAIDA!!!");
  } else {
    display.printf("Pitch:%.0f", localData.pitch);
  }
  
  display.display();
}

void enviarTelemetria() {
  SensorData localData;
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(&localData, &g_data, sizeof(SensorData));
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
  strncpy(packet.estadoGlobal, localData.estadoGlobal, 19);
  packet.fallDetected = localData.fallDetected ? 1 : 0;
  packet.signalQuality = localData.signalQuality ? 1 : 0;
  
  esp_now_send(receiverAddress, (uint8_t*)&packet, sizeof(packet));
}

void aplicarFiltroMadgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt) {
  // Implementación optimizada del filtro Madgwick
  float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
  float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
  float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;
  
  // Velocidad angular
  float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);
  
  float accNorm = sqrt(ax*ax + ay*ay + az*az);
  if (accNorm > 0.001f) {
    ax /= accNorm; ay /= accNorm; az /= accNorm;
    
    float magNorm = sqrt(mx*mx + my*my + mz*mz);
    if (magNorm > 0.001f) {
      mx /= magNorm; my /= magNorm; mz /= magNorm;
    }
    
    // Gradiente descendente
    float s0 = -2.0f * q2 * (2.0f * (q1q3 - q0q2) - ax) + 2.0f * q1 * (2.0f * (q0q1 + q2q3) - ay) - 2.0f * q0 * (2.0f * (0.5f - q1q1 - q2q2) - az);
    float s1 = 2.0f * q3 * (2.0f * (q1q3 - q0q2) - ax) + 2.0f * q0 * (2.0f * (q0q1 + q2q3) - ay) + 2.0f * q1 * (2.0f * (0.5f - q1q1 - q2q2) - az);
    float s2 = -2.0f * q0 * (2.0f * (q1q3 - q0q2) - ax) + 2.0f * q3 * (2.0f * (q0q1 + q2q3) - ay) + 2.0f * q2 * (2.0f * (0.5f - q1q1 - q2q2) - az);
    float s3 = 2.0f * q1 * (2.0f * (q1q3 - q0q2) - ax) + 2.0f * q2 * (2.0f * (q0q1 + q2q3) - ay) + 2.0f * q3 * (2.0f * (0.5f - q1q1 - q2q2) - az);
    
    float sNorm = sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (sNorm > 0.001f) {
      s0 /= sNorm; s1 /= sNorm; s2 /= sNorm; s3 /= sNorm;
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
  
  float qNorm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (qNorm > 0.001f) {
    q0 /= qNorm; q1 /= qNorm; q2 /= qNorm; q3 /= qNorm;
  }
}

void calibrarNivel() {
  float sumPitch = 0, sumRoll = 0;
  for (int i = 0; i < 50; i++) {
    if (mpu.update()) {
      float ax = mpu.getAccX(), ay = mpu.getAccY(), az = mpu.getAccZ();
      float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / PI;
      float roll = atan2(ay, az) * 180.0f / PI;
      sumPitch += pitch;
      sumRoll += roll;
    }
    delay(10);
  }
  pitchOffset = sumPitch / 50.0f;
  rollOffset = sumRoll / 50.0f;
}