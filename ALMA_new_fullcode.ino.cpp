//
// ********************************************************************
// *                                                                  *
// *  ALMA - Asistente Logístico de Monitoreo Avanzado (v1000%)       *
// *                                                                  *
// *  Nivel Élite - WRO 2026                                         *
// *                                                                  *
// *  Características:                                                *
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
// *                                                                  *
// ********************************************************************
//

#include <Wire.h>
// MAX30102 PPG Sensor
#include "MAX30105.h"
#include "spo2_algorithm.h"
// OLED 0.96
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// MLX90614 IR Thermometer (Descomentar si se usa)
// #include <Adafruit_MLX90614.h>
#include <Adafruit_BME280.h>
#include "MPU9250.h"

// --- Configuración de Pantalla OLED ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

// --- Configuración de Pines I2C para ESP32 (General) ---
#define I2C_SDA 21
#define I2C_SCL 22

// --- Configuración del MAX30102 ---
#define MAX30102_SAMPLES 100      // Tamaño del buffer para el algoritmo SpO2
#define RATE_SIZE 4                // Tamaño del buffer para el filtro de media móvil de BPM
#define SPO2_SIZE 4                // Tamaño del buffer para el filtro de media móvil de SpO2
#define SIGNAL_QUALITY_THRESHOLD 50000 // Umbral para considerar buena señal PPG

// --- Umbrales y Configuración de Caídas ---
#define FALL_IMPACT_THRESHOLD 3.5  // Umbral de impacto (Fuerza G)
#define FALL_INACTIVITY_TIME 3000  // Tiempo de inactividad post-impacto (ms)
#define FALL_ANGLE_THRESHOLD 50     // Ángulo para confirmar caída (grados)

// --- Configuración de Temporizadores ---
const unsigned long LORA_INTERVAL = 5000;   // Enviar datos cada 5 segundos (para pruebas)
const unsigned long OLED_INTERVAL = 200;    // Actualizar OLED cada 200ms (reduce parpadeo)
const unsigned long WATCHDOG_TIMEOUT = 1000; // Timeout del watchdog (1 segundo)

// --- Configuración de Low Power Mode ---
const unsigned long INACTIVITY_TIMEOUT = 300000; // 5 minutos sin movimiento para low power
const float MOTION_THRESHOLD = 0.1;              // Umbral de movimiento para despertar (G's)
unsigned long lastMotionTime = 0;
bool lowPowerMode = false;

// --- Instancias de Sensores (Globales pero protegidas por Mutex) ---
MAX30105 particleSensor;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MPU9250 mpu;
Adafruit_BME280 bme;

// --- Variables de Estado de Sensores (para modo seguro) ---
bool bme_present = false;
bool mpu_present = false;
bool max_present = false;
bool oled_present = false;

// --- Sincronización FreeRTOS ---
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t dataMutex;      // Mutex específico para proteger el struct g_data
TaskHandle_t biometricTaskHandle = NULL;
TaskHandle_t fallTaskHandle = NULL;

// --- ESTRUCTURA DE DATOS GLOBAL ---
typedef struct {
  // Biométricos
  int32_t heartRate;
  int32_t spo2;
  float temperature;        // Temperatura corporal (MLX90614)
  float humidity;
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
  bool signalQuality;       // Buena señal del MAX30102?
} SensorData;

volatile SensorData g_data;

// --- Buffers y Filtros para MAX30102 ---
uint32_t irBuffer[MAX30102_SAMPLES];
uint32_t redBuffer[MAX30102_SAMPLES];
int32_t spo2_buffer[SPO2_SIZE];
int32_t hr_buffer[RATE_SIZE];
uint8_t spo2_buffer_index = 0;
uint8_t hr_buffer_index = 0;

// --- MEJORA ELITE: Filtro de outliers (sanity check) ---
int32_t lastValidHR = 70;       // Valor inicial típico
int32_t lastValidSpO2 = 97;     // Valor inicial típico
const float MAX_HR_CHANGE = 0.10; // 10% de cambio máximo permitido

// --- Variables para el Filtro Madgwick (Orientación) ---
float beta = 0.1f;  // Ganancia del filtro Madgwick
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // Cuaternión
float pitchOffset = 0.0f;  // MEJORA ELITE: Offset para calibración de nivel
float rollOffset = 0.0f;

// --- Watchdog por Software ---
unsigned long lastTaskReset[2] = {0, 0}; // 0: BiometricTask, 1: Loop (Core1)

// --- Prototipos de Funciones ---
void actualizarOLED();
void enviarTelemetria();
void filtroCaidas(float mag, float inclination);
void aplicarFiltroMadgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt);
void resetI2CBus();
void checkWatchdog();
void inicializarFiltroOrientacion();
void calibrarNivel();  // MEJORA ELITE: Calibración dinámica
bool sanityCheckHR(int32_t newHR);  // MEJORA ELITE: Filtro de outliers
bool sanityCheckSpO2(int32_t newSpO2);
void checkLowPowerMode(float magnitude);  // MEJORA ELITE: Low Power Mode

// ******************************
// * TAREA CORE 0: BIOMETRÍA   *
// ******************************
void BiometricTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // ~50Hz de muestreo para el MAX30102

  // Variables para el algoritmo SpO2
  int32_t spo2, hr;
  int8_t validSPO2, validHR;
  uint8_t samplesCollected = 0;
  uint32_t signalQualitySum = 0;

  Serial.println("[Tarea0] Biometría iniciada.");

  for (;;) {
    // --- 1. RESET DEL WATCHDOG ---
    lastTaskReset[0] = millis();

    // --- 2. VERIFICAR MODO LOW POWER ---
    if (lowPowerMode) {
      // En low power, muestreamos menos frecuente
      vTaskDelay(pdMS_TO_TICKS(100)); // Reducir frecuencia a 10Hz
      
      // Intentar leer pero sin forzar
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (max_present) {
          particleSensor.check();
        }
        xSemaphoreGive(i2cMutex);
      }
      continue;
    }

    // --- 3. LLENADO DEL BUFFER CIRCULAR (Modo normal) ---
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (max_present) {
        particleSensor.check();

        while (particleSensor.available() && samplesCollected < MAX30102_SAMPLES) {
          redBuffer[samplesCollected] = particleSensor.getRed();
          irBuffer[samplesCollected] = particleSensor.getIR();
          
          // Acumular para calcular calidad de señal
          if (samplesCollected > 0) {
            signalQualitySum += irBuffer[samplesCollected];
          }
          
          particleSensor.nextSample();
          samplesCollected++;
        }
      }
      xSemaphoreGive(i2cMutex);
    }

    // --- 4. PROCESAMIENTO DEL ALGORITMO ---
    if (samplesCollected >= MAX30102_SAMPLES) {
      // Calcular calidad de señal (MEJORA ELITE: barra de confianza)
      float avgSignal = signalQualitySum / MAX30102_SAMPLES;
      bool goodSignal = (avgSignal > SIGNAL_QUALITY_THRESHOLD);
      
      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        g_data.signalQuality = goodSignal;
        xSemaphoreGive(dataMutex);
      }

      // Ejecutar el algoritmo pesado (solo si hay señal)
      if (goodSignal) {
        maxim_heart_rate_and_oxygen_saturation(irBuffer, MAX30102_SAMPLES, redBuffer, &spo2, &validSPO2, &hr, &validHR);

        // MEJORA ELITE: Sanity check para BPM
        if (validHR && hr > 20 && hr < 220) {
          if (sanityCheckHR(hr)) {
            hr_buffer[hr_buffer_index] = hr;
            hr_buffer_index = (hr_buffer_index + 1) % RATE_SIZE;
            int32_t hr_sum = 0;
            for (int i = 0; i < RATE_SIZE; i++) hr_sum += hr_buffer[i];
            int32_t filteredHR = hr_sum / RATE_SIZE;
            
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              g_data.heartRate = filteredHR;
              lastValidHR = filteredHR;
              xSemaphoreGive(dataMutex);
            }
          }
        }

        // MEJORA ELITE: Sanity check para SpO2
        if (validSPO2 && spo2 > 70 && spo2 < 100) {
          if (sanityCheckSpO2(spo2)) {
            spo2_buffer[spo2_buffer_index] = spo2;
            spo2_buffer_index = (spo2_buffer_index + 1) % SPO2_SIZE;
            int32_t spo2_sum = 0;
            for (int i = 0; i < SPO2_SIZE; i++) spo2_sum += spo2_buffer[i];
            int32_t filteredSpO2 = spo2_sum / SPO2_SIZE;
            
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              g_data.spo2 = filteredSpO2;
              lastValidSpO2 = filteredSpO2;
              xSemaphoreGive(dataMutex);
            }
          }
        }
      }

      samplesCollected = 0;
      signalQualitySum = 0;
    }

    // --- 5. LECTURA DE SENSORES AMBIENTALES ---
    static unsigned long lastEnvRead = 0;
    if (millis() - lastEnvRead > 1000 && !lowPowerMode) {
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (bme_present) {
          float newHum = bme.readHumidity();
          float newPres = bme.readPressure() / 100.0F;
          if (!isnan(newHum) && !isnan(newPres)) {
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              g_data.humidity = newHum;
              g_data.pressure = newPres;
              xSemaphoreGive(dataMutex);
            }
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
// * SETUP (CORE 1)             *
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

  Serial.println("[SETUP] Listo. Entrando en loop principal.");
}

// ******************************
// * LOOP PRINCIPAL (CORE 1)    *
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

      // MEJORA ELITE: Calcular dt de forma segura
      if (currentMicros > lastMPURead) {
        dt = (currentMicros - lastMPURead) / 1000000.0f;
        // Limitar dt a valores razonables (0.001s a 0.1s)
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.1f) dt = 0.1f;
      }

      // Aplicar filtro Madgwick
      aplicarFiltroMadgwick(ax, ay, az, gx, gy, gz, mx, my, mz, dt);

      // Calcular magnitud y aplicar offset de calibración
      float mag = sqrt(ax*ax + ay*ay + az*az);
      float pitch = (asin(2.0f*(q0*q2 - q3*q1)) * 180.0f / PI) - pitchOffset;
      float roll = (atan2(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 180.0f / PI) - rollOffset;

      // MEJORA ELITE: Verificar movimiento para Low Power
      checkLowPowerMode(mag);

      // Actualizar estructura de datos global
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
        g_data.yaw = atan2(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 180.0f / PI;
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
  
  if (!lowPowerMode) {  // No detectar caídas en low power
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
      // En low power, mantener pantalla apagada
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

// ******************************
// * IMPLEMENTACIÓN DE FUNCIONES *
// ******************************

// --- MEJORA ELITE: Calibración de nivel ---
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
        
        sumPitch += asin(2.0f*(q0*q2 - q3*q1)) * 180.0f / PI;
        sumRoll += atan2(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 180.0f / PI;
      }
      xSemaphoreGive(i2cMutex);
    }
    delay(10);
  }
  
  pitchOffset = sumPitch / samples;
  rollOffset = sumRoll / samples;
  
  Serial.printf("[Calibracion] Offset - Pitch: %.2f, Roll: %.2f\n", pitchOffset, rollOffset);
}

// --- MEJORA ELITE: Sanity Check para HR ---
bool sanityCheckHR(int32_t newHR) {
  if (lastValidHR == 0) return true; // Primera lectura
  
  float change = abs(newHR - lastValidHR) / (float)lastValidHR;
  return (change <= MAX_HR_CHANGE);
}

// --- MEJORA ELITE: Sanity Check para SpO2 ---
bool sanityCheckSpO2(int32_t newSpO2) {
  if (lastValidSpO2 == 0) return true;
  
  float change = abs(newSpO2 - lastValidSpO2) / (float)lastValidSpO2;
  return (change <= MAX_HR_CHANGE); // Mismo umbral
}

// --- MEJORA ELITE: Low Power Mode ---
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

// --- INICIALIZACIÓN DEL FILTRO DE ORIENTACIÓN ---
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
