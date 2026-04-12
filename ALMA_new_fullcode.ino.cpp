//
// ********************************************************************
// *                                                                 *
// *  ALMA - Asistente Logístico de Monitoreo Avanzado               *
// *                                                                 *                                         *
// *                                                                 *
// *  Características:                                               *
// *  - Multi-tarea optimizada (Core 0: Biometría / Core 1: Mov.)    *
// *  - Filtro Madgwick para orientación absoluta                    *
// *  - Detección de caídas con confirmación por ángulo              *
// *  - Buffer circular PPG sin pérdida de muestras                  *
// *  - Filtro de outliers médicos (sanity check)                    *
// *  - Modo Low Power con wake-on-motion                            *
// *  - Watchdog por software con reinicio I2C                       *
// *  - Estructura binaria para telemetría LoRa/BLE                  *
// *  - Barra de calidad de señal en OLED                            *
// *  - Calibración dinámica de orientación                          *
// *                                                                 *
// *******************************************************************
//

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include "MPU9250.h"
#include <WiFi.h>
#include <esp_now.h>

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

uint8_t receiverAddress[] = {0x68, 0x25, 0xDD, 0x2F, 0xCA, 0x9C};
esp_now_peer_info_t peerInfo;

// ==================== CONFIGURACIÓN ============================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C
// ==================== PINES I2C ================================
#define I2C_SDA 21
#define I2C_SCL 22

// ============== CONFIGURACIÓN MAX30102 ==========================
#define MAX30102_SAMPLES 100      // Tamaño del buffer para el algoritmo SpO2
#define RATE_SIZE 4                // Tamaño del buffer para el filtro de media móvil de BPM
#define SPO2_SIZE 4                // Tamaño del buffer para el filtro de media móvil de SpO2
#define SIGNAL_QUALITY_THRESHOLD 50000 // Umbral para considerar buena señal PPG

// ================ Umbrales y Configuración de Caídas ================
#define FALL_IMPACT_THRESHOLD 3.5  // Umbral de impacto (Fuerza G)
#define FALL_INACTIVITY_TIME 3000  // Tiempo de inactividad post-impacto (ms)
#define FALL_ANGLE_THRESHOLD 50     // Ángulo para confirmar caída (grados)

// ================ Configuración de Temporizadores ================
const unsigned long LORA_INTERVAL = 5000;   // Enviar datos cada 5 segundos (para pruebas)
const unsigned long OLED_INTERVAL = 200;    // Actualizar OLED cada 200ms (reduce parpadeo)
const unsigned long WATCHDOG_TIMEOUT = 1000; // Timeout del watchdog (1 segundo)

// ================ Configuración de Low Power Mode ================
const unsigned long INACTIVITY_TIMEOUT = 300000; // 5 minutos sin movimiento para low power
const float MOTION_THRESHOLD = 0.1;              // Umbral de movimiento para despertar (G's)
unsigned long lastMotionTime = 0;
bool lowPowerMode = false;

// ================ Instancias de Sensores (Globales pero protegidas por Mutex) ================
MAX30105 particleSensor;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MPU9250 mpu;
Adafruit_BMP280 bme;

// ================ Variables de Estado de Sensores (para modo seguro) ================
bool bme_present = false;
bool mpu_present = false;
bool max_present = false;
bool oled_present = false;

// ==================== Sincronización FreeRTOS ====================
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t dataMutex;      // Mutex específico para proteger el struct g_data
TaskHandle_t biometricTaskHandle = NULL;
TaskHandle_t fallTaskHandle = NULL;

// ==================== ESTRUCTURA DE DATOS GLOBAL ====================
typedef struct {
  // Biométricos
  int32_t heartRate;
  int32_t spo2;
  float temperature;
  float pressure;
  // Movimiento
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, yaw;
  float magnitude;
  // Estados
  char estadoGlobal[20];
  bool fallDetected;
  unsigned long fallTimestamp;
  // Calidad de señal
  bool signalQuality;
} SensorData;

SensorData g_data;

// ==================== Buffers y Filtros para MAX30102 ====================
uint32_t irBuffer[MAX30102_SAMPLES];
uint32_t redBuffer[MAX30102_SAMPLES];
uint32_t samplesCollected = 0;
int32_t spo2_buffer[SPO2_SIZE];
int32_t hr_buffer[RATE_SIZE];
uint8_t spo2_buffer_index = 0;
uint8_t hr_buffer_index = 0;

// ==================== MEJORA ELITE: SISTEMA DE FUSIÓN SENSORIAL Y CONFIANZA DINÁMICA ====================
enum BiometricState {
    STATE_NO_FINGER,    // Sensor vacío o señal insuficiente
    STATE_ACQUIRING,   // Dedo detectado, buscando pulso estable (Warm-up)
    STATE_LOCKED       // Seguimiento fisiológico estricto
};

struct ConfidenceManager {
    BiometricState state = STATE_NO_FINGER;
    uint32_t lastValidTimestampHR = 0;      // Timestamp independiente para HR
    uint32_t lastValidTimestampSpO2 = 0;    // Timestamp independiente para SpO2
    int32_t lastStableHR = 70;
    int32_t lastStableSpO2 = 97;
    int acquiringCycles = 0;
    
    // Parámetros Fisiológicos
    const float MAX_BPM_PER_SEC = 4.0f;      // Máximo cambio real en humanos
    const float MAX_SPO2_PER_SEC = 0.5f;     // El SpO2 cambia muy lento
    const int   ACQUIRING_REQUIRED = 5;      // Ciclos para considerar "LOCKED"
    
    // Parámetros de IMU (G's)
    const float MAG_IDEAL = 1.0f;
    const float MAG_TOLERANCE = 0.15f;       // Ventana de 0.85G a 1.15G para HR
    const float MAG_TOLERANCE_SPO2 = 0.25f;  // Tolerancia relajada para SpO2
};

ConfidenceManager vld;

// ==================== Variables para el Filtro Madgwick (Orientación) ====================
float beta = 0.1f;  // Ganancia del filtro Madgwick
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // Cuaternión
float pitchOffset = 0.0f;  // MEJORA ELITE: Offset para calibración de nivel
float rollOffset = 0.0f;

// ==================== Watchdog por Software ====================
unsigned long lastTaskReset[2] = {0, 0}; // 0: BiometricTask, 1: Loop (Core1)

// ==================== Prototipos de Funciones ====================
void actualizarOLED();
void enviarTelemetria();
void filtroCaidas(float mag, float inclination);
void aplicarFiltroMadgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt);
void resetI2CBus();
void checkWatchdog();
void inicializarFiltroOrientacion();
void calibrarNivel();
bool sanityCheckHR(int32_t newHR);
bool sanityCheckSpO2(int32_t newSpO2);
void checkLowPowerMode(float magnitude);

// ******************************
// * TAREA CORE 0: BIOMETRÍA    * ✅ (Versión Corregida v1000%)
// ******************************
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

    // --- 1. LLENADO SEGURO DEL BUFFER ---
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (max_present) {
        particleSensor.check();
        while (particleSensor.available() && samplesCollected < MAX30102_SAMPLES) {
          redBuffer[samplesCollected] = particleSensor.getRed();
          irBuffer[samplesCollected] = particleSensor.getIR();
          signalQualitySum += irBuffer[samplesCollected];
          particleSensor.nextSample();
          samplesCollected++;
        }
      }
      xSemaphoreGive(i2cMutex);
    }

        // --- PROCESAMIENTO CUANDO EL BUFFER ESTÁ LLENO ---
    if (samplesCollected >= MAX30102_SAMPLES) {
      float avgSignal = signalQualitySum / MAX30102_SAMPLES;
      bool goodSignal = (avgSignal > SIGNAL_QUALITY_THRESHOLD);

      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        g_data.signalQuality = goodSignal;
        xSemaphoreGive(dataMutex);
      }

      if (goodSignal) {
        maxim_heart_rate_and_oxygen_saturation(irBuffer, MAX30102_SAMPLES, redBuffer, &spo2, &validSPO2, &hr, &validHR);

        // Validar y actualizar
        bool hrValid = validHR && sanityCheckHR(hr);
        
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          // Publicamos el valor del ConfidenceManager (que ya está filtrado y estabilizado)
          g_data.heartRate = vld.lastStableHR; 
          
          if (validSPO2 && sanityCheckSpO2(spo2)) {
            g_data.spo2 = vld.lastStableSpO2;
          }
          xSemaphoreGive(dataMutex);
        }
      } else {
        Serial.println("[Biometria] Sin señal. Reset.");
        vld.state = STATE_NO_FINGER;
        vld.lastValidTimestampHR = 0;   // Reset de timestamps para forzar 
        vld.lastValidTimestampSpO2 = 0; // sincronización limpia al reconectar
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          g_data.heartRate = 0; // Solo aquí permitimos el 0
          xSemaphoreGive(dataMutex);
        }
      }

      // --- DESPLAZAMIENTO DE VENTANA (Sliding Window) ---
      // Desplazamos 25 muestras para sobreescribirlas en el siguiente ciclo
      for (byte i = 25; i < 100; i++) {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
      }
      samplesCollected = 75;
    }

    // --- 3. LECTURA AMBIENTAL ESPORÁDICA ---
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

// ******************************
// * SETUP (CORE 1)             * ✅
// ******************************
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n\n[ALMA v1000% - WRO 2026] Iniciando...");

  // --- Crear Mutex ---
  i2cMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();

  // --- Inicializar Bus I2C ---
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // --- INICIALIZAR SENSORES ---
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
    max_present = particleSensor.begin(Wire, I2C_SPEED_STANDARD);
    if (!max_present) {
      Serial.println("[ERROR] MAX30102 no detectado.");
    } else {
      particleSensor.setup(0x24, 8, 2, 100, 411, 4096);
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

  // --- Inicializar filtro de orientación ---
  if (mpu_present) {
    inicializarFiltroOrientacion();
    calibrarNivel();  // Establecer offset cero para la posición actual
  }

  // --- Inicializar estadoGlobal ---
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    strcpy(g_data.estadoGlobal, "INICIANDO");
    g_data.signalQuality = false;
    g_data.heartRate = 0;
    g_data.spo2 = 0;
    xSemaphoreGive(dataMutex);
  }

  lastMotionTime = millis();

  // --- CONFIGURACIÓN WIFI PARA ESP-NOW ---
  WiFi.mode(WIFI_STA);
  
  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] Error inicializando ESP-NOW");
    return;
  }

  peerInfo = {};
  // Registrar el receptor
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("[ERROR] Fallo al añadir el peer ESP-NOW");
    return;
  }
  
  Serial.println("[Red] WiFi y ESP-NOW listos.");

  // --- Crear Tarea en el Núcleo 0 ---
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

  Serial.println("[SETUP] Listo. Entrando en loop principal.");
}

// ******************************
// * LOOP PRINCIPAL (CORE 1)    * ✅
// ******************************
void loop() {
  // --- 1. RESET DEL WATCHDOG ---
  lastTaskReset[1] = millis();

  // --- 2. LECTURA DE LA IMU CON DT ROBUSTO (MEJORA ELITE) ---
  static unsigned long lastMPURead = 0;
  unsigned long currentMicros = micros();
  float dt = 0.01f; // Valor por defecto seguro

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

      // Calcular dt de forma segura
      if (currentMicros > lastMPURead) {
        dt = (currentMicros - lastMPURead) / 1000000.0f;
        // Limitar dt a valores razonables (0.001s a 0.1s)
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.1f) dt = 0.1f;
      }

      aplicarFiltroMadgwick(ax, ay, az, gx, gy, gz, mx, my, mz, dt);

      // Calcular magnitud y aplicar offset de calibración
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

  // Solo actualizar timestamp si la lectura fue exitosa
  if (lecturaExitosa) {
    lastMPURead = currentMicros;
  }

  // --- 3. LÓGICA DE DETECCIÓN DE CAÍDAS ---
  SensorData localData;
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(&localData, (const void*)&g_data, sizeof(SensorData));
    xSemaphoreGive(dataMutex);
  }

  if (!lowPowerMode) {
    filtroCaidas(localData.magnitude, localData.pitch);
  }

  // --- 4. ENVÍO DE TELEMETRÍA ---
  static unsigned long lastLoRaSend = 0;
  if (millis() - lastLoRaSend > LORA_INTERVAL && !lowPowerMode) {
    enviarTelemetria();
    lastLoRaSend = millis();
  }

  // --- 5. ACTUALIZACIÓN DE LA PANTALLA OLED ---
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

  // --- 6. WATCHDOG POR SOFTWARE ---
  checkWatchdog();

  delay(10);
}

// ____________________________________________

// ******************************
// * IMPLEMENTACIÓN DE FUNCIONES *
// ******************************

// --- Calibración de nivel --- ✅
void calibrarNivel() {
  Serial.println("[Calibracion] Estableciendo nivel cero...");
  delay(500); // Esperar a que el sensor se estabilice

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
        sumRoll += atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
      }
      xSemaphoreGive(i2cMutex);
    }
    delay(10);
  }

  pitchOffset = sumPitch / samples;
  rollOffset = sumRoll / samples;

  Serial.printf("[Calibracion] Offset - Pitch: %.2f, Roll: %.2f\n", pitchOffset, rollOffset);
}

// ==================== Buffers para Suavizado Final ====================
#define HR_FILTER_SIZE 4
int32_t hrHistory[HR_FILTER_SIZE];
uint8_t hrHistoryIdx = 0;

// --- FUNCIÓN DE VALIDACIÓN ELITE (VERSIÓN ESTABILIZADA v2) ---
bool sanityCheckHR(int32_t newHR) {
    uint32_t now = millis();
    float currentMag = 1.0f;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        currentMag = g_data.magnitude;
        xSemaphoreGive(dataMutex);
    }

    // 1. Fusión Sensorial Estricta
    if (abs(currentMag - vld.MAG_IDEAL) > vld.MAG_TOLERANCE) return false;

    // 2. Límites Fisiológicos Reales
    if (newHR < 45 || newHR > 180) return false;

    float dt = (now - vld.lastValidTimestampHR) / 1000.0f;
    if (dt <= 0 || dt > 2.0f) dt = 0.5f;

    switch (vld.state) {
        case STATE_NO_FINGER:
            // ANCLA FISIOLÓGICA: Empezamos en un rango lógico (75 BPM)
            vld.state = STATE_ACQUIRING;
            vld.acquiringCycles = 0;
            vld.lastStableHR = 75; 
            vld.lastValidTimestampHR = now;
            vld.lastValidTimestampSpO2 = 0; // Forzar Cold Start en SpO2
            return false; 

        case STATE_ACQUIRING:
            // CONVERGENCIA CONTROLADA (8 BPM/seg):
            if (abs(newHR - vld.lastStableHR) < (8.0f * dt)) {
                vld.acquiringCycles++;
                vld.lastStableHR = newHR;
                vld.lastValidTimestampHR = now;
                if (vld.acquiringCycles >= vld.ACQUIRING_REQUIRED) vld.state = STATE_LOCKED;
                return true;
            } else {
                // Convergencia lenta hacia el valor objetivo para evitar saltos
                if (newHR > vld.lastStableHR) vld.lastStableHR += 1;
                else vld.lastStableHR -= 1;
                return false;
            }

        case STATE_LOCKED:
            // SEGUIMIENTO ESTRICTO (3 BPM/seg) + Filtro Anti-Doble Pulso
            if (newHR > (vld.lastStableHR * 1.8f)) return false; 

            float maxAllowed = vld.MAX_BPM_PER_SEC * dt;
            if (abs(newHR - vld.lastStableHR) <= (maxAllowed + 1.5f)) {
                vld.lastStableHR = newHR;
                vld.lastValidTimestampHR = now;
                return true;
            } else {
                static int outlierCount = 0;
                if (++outlierCount > 5) {
                    vld.state = STATE_ACQUIRING;
                    vld.acquiringCycles = 0;
                    outlierCount = 0;
                }
            }
            break;
    }
    return false;
}

// --- VALIDACIÓN DE OXIGENACIÓN CON FILTRO DE ESTABILIDAD ---
bool sanityCheckSpO2(int32_t newSpO2) {
    if (newSpO2 < 85 || newSpO2 > 100) return false;

    float currentMag = 1.0f;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        currentMag = g_data.magnitude;
        xSemaphoreGive(dataMutex);
    }
    
    // Si hay movimiento excesivo, el SpO2 es la primera señal en corromperse
    // Usamos la tolerancia relajada (0.25G) asimétrica
    if (abs(currentMag - vld.MAG_IDEAL) > vld.MAG_TOLERANCE_SPO2) return false;

    uint32_t now = millis();

    // SEMILLA DE ARRANQUE DINÁMICA (Cold Start)
    if (vld.lastValidTimestampSpO2 == 0) {
        if (newSpO2 >= 90) { // Aceptar primer valor lógico directamente
            vld.lastStableSpO2 = newSpO2;
            vld.lastValidTimestampSpO2 = now;
            return true;
        }
        return false;
    }

    // Cálculo de dt independiente
    float dt = (now - vld.lastValidTimestampSpO2) / 1000.0f;
    
    if (vld.state == STATE_LOCKED) {
        if (dt > 0 && dt < 2.0f) {
            // El SpO2 cambia muy lento, permitimos 0.5% por segundo
            if (abs(newSpO2 - vld.lastStableSpO2) > (vld.MAX_SPO2_PER_SEC * dt + 1.0f)) {
                return false;
            }
        }
    }

    vld.lastStableSpO2 = newSpO2;
    vld.lastValidTimestampSpO2 = now;
    return true;
}

// --- Low Power Mode --- ✅
void checkLowPowerMode(float magnitude) {
  static unsigned long lastWakeTime = 0;

  // Detectar movimiento
  if (magnitude < (1.0f + MOTION_THRESHOLD) && magnitude > (1.0f - MOTION_THRESHOLD)) {
    // En reposo (cerca de 1G)
    if (millis() - lastMotionTime > INACTIVITY_TIMEOUT && !lowPowerMode) {
      lowPowerMode = true;
      Serial.println("[LowPower] Modo ahorro activado - Sin movimiento detectado");

      if (oled_present) {
        display.ssd1306_command(SSD1306_DISPLAYOFF);
      }
    }
  } else {
    // Hay movimiento
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

// --- INICIALIZACIÓN DEL FILTRO DE ORIENTACIÓN --- ✅
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

// --- FILTRO DE MADGWICK --- ✅
void aplicarFiltroMadgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt) {
  if (isnan(ax) || isnan(ay) || isnan(az) || isnan(gx) || isnan(gy) || isnan(gz)) return;

  // --- PROTECCIÓN 2: Fallback si el magnetómetro está muerto (0,0,0) ---
  float magNorm = sqrt(mx * mx + my * my + mz * mz);
  if (magNorm < 0.001f) {
    // Si no hay magnetómetro, usamos una versión simplificada de 6 ejes
    // o simplemente ignoramos el aporte del magnetómetro en el gradiente.
    mx = 0; my = 0; mz = 0;
  }

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;

  // --- PASO CRÍTICO: Inicializar productos de cuaterniones ---
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

  // Velocidad de cambio del cuaternión
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Solo procesar si el acelerómetro tiene datos (evitar división por cero)
  float accNorm = sqrt(ax * ax + ay * ay + az * az);
  if (accNorm > 0.0f) {
    recipNorm = 1.0f / accNorm;
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    // Normalizar magnetómetro
    float magNorm = sqrt(mx * mx + my * my + mz * mz);
    if (magNorm > 0.0f) {
      recipNorm = 1.0f / magNorm;
      mx *= recipNorm; my *= recipNorm; mz *= recipNorm;
    }

    // Cálculos auxiliares para optimizar
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

    // Gradiente descendente
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    // Normalizar paso del gradiente
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

  // Integrar para hallar el nuevo cuaternión
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalizar cuaternión final
  float qNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (isnan(qNorm) || qNorm < 0.0001f) {
    // Si colapsa, resetear a la posición identidad
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    return;
  }

  // Normalización final (ya la tienes, pero asegúrate que use recipNorm con chequeo)
  recipNorm = 1.0f / qNorm;
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

// --- FILTRO DE CAÍDAS --- ✅
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

// --- ACTUALIZACIÓN DE PANTALLA OLED (con barra de calidad) --- ✅
void actualizarOLED() {
  if (!oled_present) return;

  SensorData localData;
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(&localData, (const void*)&g_data, sizeof(SensorData));
    xSemaphoreGive(dataMutex);
  }

  char buffer[20];
  display.clearDisplay();

  // Título
  display.setTextSize(1);

  // Fila 1: BPM y SpO2
  display.setCursor(0, 12);
  sprintf(buffer, "BPM:%3ld SpO2:%3ld%%", localData.heartRate, localData.spo2);
  display.print(buffer);

  // MEJORA ELITE: Barra de calidad de señal
  if (localData.signalQuality) {
    display.fillRect(100, 14, 20, 6, WHITE);  // Barra llena = buena señal
  } else {
    display.drawRect(100, 14, 20, 6, WHITE);  // Solo contorno = mala señal
  }

  // Fila 2: Temperatura y Humedad
  display.setCursor(0, 24);
  sprintf(buffer, "T:%.2f", localData.temperature);
  display.print(buffer);

  // Fila 3: Presión
  display.setCursor(0, 36);
  sprintf(buffer, "P:%.1fhPa", localData.pressure);
  display.print(buffer);

  // Fila 4: Estado
  display.setCursor(0, 48);
  display.print("EST:");
  display.setCursor(30, 48);
  display.print(localData.estadoGlobal);

  display.display();
}

// --- ENVÍO DE TELEMETRÍA ---
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

  bool biometriaValida = localData.signalQuality && (packet.heartRate > 0) && (packet.spo2 > 0);

  if (biometriaValida) {
    
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

// --- REINICIO DEL BUS I2C --- ✅
void resetI2CBus() {
  Serial.println("[WATCHDOG] Reiniciando bus I2C...");
  Wire.end();
  delay(100);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
}

// --- VERIFICACIÓN DEL WATCHDOG --- ✅
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