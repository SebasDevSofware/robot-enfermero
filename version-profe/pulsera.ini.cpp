// ===================================================================
// | PULSERA BIOCHRONORITMIC - EMISOR ESP-NOW ULTRA RÁPIDO           |
// | Modo: WIFI_STA - Canal fijo 1 - Sin servidor web               |
// | Versión: FINAL - 100% COMUNICACIÓN EFECTIVA                     |
// ===================================================================

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_BMP280.h>
#include "MPU9250.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "esp_mac.h"

// ==================== CONFIGURACIÓN ====================
#define I2C_SDA 21
#define I2C_SCL 22
#define ESP_NOW_CHANNEL 1                    // ¡CRÍTICO! Canal fijo 1
#define MAX30102_SAMPLES 100
#define SIGNAL_QUALITY_THRESHOLD 50000

// === MAC ADDRESS DEL ROBOT (¡CAMBIA ESTO POR LA MAC REAL DE TU ROBOT!) ===
// Para obtener la MAC: sube el código temporal al robot y copia el valor
uint8_t robotMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // ¡CAMBIA AQUÍ!

// ==================== ESTRUCTURA DE DATOS BINARIA ====================
// IMPORTANTE: Debe ser IDÉNTICA en ambos códigos
typedef struct __attribute__((packed)) {
  uint32_t timestamp;      // 4 bytes - Tiempo del dato
  int32_t heartRate;       // 4 bytes - Latidos por minuto
  int32_t spo2;            // 4 bytes - Saturación de oxígeno (%)
  float temperature;       // 4 bytes - Temperatura (°C)
  float pressure;          // 4 bytes - Presión atmosférica (hPa)
  float roll;              // 4 bytes - Rotación en eje X (grados)
  float pitch;             // 4 bytes - Rotación en eje Y (grados)
  float yaw;               // 4 bytes - Rotación en eje Z (grados)
  float magnitude;         // 4 bytes - Magnitud de movimiento (G)
  uint8_t fallDetected;    // 1 byte  - ¿Caída detectada?
  uint8_t signalQuality;   // 1 byte  - Calidad de señal (0-100)
  uint8_t reserved[2];     // 2 bytes - Padding para alinear
} TelemetryPacket;         // TOTAL: 40 bytes

// ==================== VARIABLES GLOBALES ====================
MAX30105 particleSensor;
Adafruit_BMP280 bme;
MPU9250 mpu;

bool bme_present = false;
bool mpu_present = false;
bool max_present = false;

TelemetryPacket packet;
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t dataMutex;

// Buffers para MAX30102
uint32_t irBuffer[MAX30102_SAMPLES];
uint32_t redBuffer[MAX30102_SAMPLES];
uint32_t samplesCollected = 0;

// Variables de estado biométrico
int32_t lastStableHR = 75;
int32_t lastStableSpO2 = 97;
bool signalGood = false;

// Filtro Madgwick para IMU
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float pitchOffset = 0.0f, rollOffset = 0.0f;
unsigned long lastMPURead = 0;

// Estadísticas de comunicación
uint32_t packetsSent = 0;
uint32_t packetsAcked = 0;
uint32_t packetsFailed = 0;
unsigned long lastStatsLog = 0;

// Control de envío dinámico
unsigned long lastSendTime = 0;
int sendInterval = 100;  // 100ms normal, 50ms si hay movimiento

// ==================== PROTOTIPOS ====================
void initSensors();
void updateOrientation(float ax, float ay, float az, float gx, float gy, float gz, 
                       float mx, float my, float mz, float dt);
void calibrateSensors();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void sendTelemetry();
void printMAC();

// ==================== CALLBACK ESP-NOW (ACK) ====================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    packetsAcked++;
  } else {
    packetsFailed++;
  }
}

// ==================== TAREA BIOMETRÍA (CORE 0 - Prioridad alta) ====================
void BiometricTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);  // 50Hz para muestreo
  
  int32_t spo2, hr;
  int8_t validSPO2, validHR;
  
  while (true) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      
      // Leer MAX30102 (oxímetro)
      if (max_present) {
        particleSensor.check();
        while (particleSensor.available() && samplesCollected < MAX30102_SAMPLES) {
          redBuffer[samplesCollected] = particleSensor.getRed();
          irBuffer[samplesCollected] = particleSensor.getIR();
          particleSensor.nextSample();
          samplesCollected++;
        }
        
        if (samplesCollected >= MAX30102_SAMPLES) {
          // Calcular calidad de señal
          uint32_t signalQualitySum = 0;
          for (int i = 0; i < MAX30102_SAMPLES; i++) signalQualitySum += irBuffer[i];
          float avgSignal = (float)signalQualitySum / MAX30102_SAMPLES;
          signalGood = (avgSignal > SIGNAL_QUALITY_THRESHOLD);
          
          if (signalGood) {
            maxim_heart_rate_and_oxygen_saturation(irBuffer, MAX30102_SAMPLES, redBuffer, 
                                                    &spo2, &validSPO2, &hr, &validHR);
            
            if (validHR && hr >= 45 && hr <= 180) {
              lastStableHR = hr;
            }
            if (validSPO2 && spo2 >= 85 && spo2 <= 100) {
              lastStableSpO2 = spo2;
            }
          }
          
          // Shift buffers (ventana deslizante de 75 muestras)
          for (byte i = 25; i < 100; i++) {
            redBuffer[i - 25] = redBuffer[i];
            irBuffer[i - 25] = irBuffer[i];
          }
          samplesCollected = 75;
        }
      }
      
      // Leer BME280 (temperatura y presión) cada 2 segundos
      static unsigned long lastEnvRead = 0;
      if (millis() - lastEnvRead > 2000) {
        if (bme_present) {
          float temp = bme.readTemperature();
          float pres = bme.readPressure() / 100.0F;
          if (!isnan(temp)) {
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              packet.temperature = temp;
              packet.pressure = pres;
              xSemaphoreGive(dataMutex);
            }
          }
        }
        lastEnvRead = millis();
      }
      
      xSemaphoreGive(i2cMutex);
    }
    
    // Actualizar datos biométricos en estructura compartida
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      packet.heartRate = signalGood ? lastStableHR : 0;
      packet.spo2 = signalGood ? lastStableSpO2 : 0;
      packet.signalQuality = signalGood ? 75 : 25;
      packet.timestamp = millis();
      xSemaphoreGive(dataMutex);
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ==================== TAREA IMU (CORE 1) ====================
void IMUTask(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      if (mpu_present && mpu.update()) {
        unsigned long now = micros();
        float dt = (now - lastMPURead) / 1000000.0f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.05f) dt = 0.01f;
        lastMPURead = now;
        
        float ax = mpu.getAccX();
        float ay = mpu.getAccY();
        float az = mpu.getAccZ();
        float gx = mpu.getGyroX();
        float gy = mpu.getGyroY();
        float gz = mpu.getGyroZ();
        float mx = mpu.getMagX();
        float my = mpu.getMagY();
        float mz = mpu.getMagZ();
        
        updateOrientation(ax, ay, az, gx, gy, gz, mx, my, mz, dt);
        
        float asinInput = 2.0f * (q0 * q2 - q3 * q1);
        if (asinInput > 1.0f) asinInput = 1.0f;
        if (asinInput < -1.0f) asinInput = -1.0f;
        
        float mag = sqrt(ax * ax + ay * ay + az * az);
        float pitch = (asin(asinInput) * 180.0f / PI) - pitchOffset;
        float roll = (atan2(2.0f * (q0 * q1 + q2 * q3), 
                     1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI) - rollOffset;
        float yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 
                   1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;
        
        // Detectar caída (impacto > 3.5G)
        bool fallDetected = (mag > 3.5f);
        
        // Ajustar intervalo de envío según movimiento
        if (mag > 2.0f) {
          sendInterval = 50;   // Más rápido si hay movimiento
        } else {
          sendInterval = 100;  // Normal
        }
        
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          packet.magnitude = mag;
          packet.roll = roll;
          packet.pitch = pitch;
          packet.yaw = yaw;
          packet.fallDetected = fallDetected ? 1 : 0;
          xSemaphoreGive(dataMutex);
        }
      }
      xSemaphoreGive(i2cMutex);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==================== TAREA ENVÍO ESP-NOW ====================
void SendTask(void *pvParameters) {
  while (true) {
    unsigned long now = millis();
    
    if (now - lastSendTime >= sendInterval) {
      sendTelemetry();
      lastSendTime = now;
    }
    
    // Log estadísticas cada 10 segundos
    if (now - lastStatsLog > 10000) {
      Serial.print("[STATS] Enviados:");
      Serial.print(packetsSent);
      Serial.print(" OK:");
      Serial.print(packetsAcked);
      Serial.print(" Fail:");
      Serial.print(packetsFailed);
      Serial.print(" Tasa:");
      if (packetsSent > 0) {
        Serial.print(100.0f * packetsAcked / packetsSent);
      } else {
        Serial.print("0");
      }
      Serial.println("%");
      lastStatsLog = now;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==================== ENVÍO DE TELEMETRÍA ====================
void sendTelemetry() {
  TelemetryPacket localPacket;
  
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(&localPacket, &packet, sizeof(TelemetryPacket));
    xSemaphoreGive(dataMutex);
  }
  
  localPacket.timestamp = millis();
  
  // Solo enviar si hay datos válidos
  if (localPacket.heartRate > 0 || localPacket.spo2 > 0) {
    esp_err_t result = esp_now_send(robotMAC, (uint8_t *)&localPacket, sizeof(TelemetryPacket));
    
    if (result == ESP_OK) {
      packetsSent++;
      // Log cada 20 paquetes
      if (packetsSent % 20 == 0) {
        Serial.print("📡 Enviado: HR=");
        Serial.print(localPacket.heartRate);
        Serial.print(" SpO2=");
        Serial.println(localPacket.spo2);
      }
    } else {
      packetsFailed++;
      Serial.print("[ERROR] Envío fallido: ");
      Serial.println(result);
    }
  }
}

// ==================== FILTRO MADGWICK (Orientación) ====================
void updateOrientation(float ax, float ay, float az, float gx, float gy, float gz, 
                       float mx, float my, float mz, float dt) {
  float beta = 0.1f;
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
  float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
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
  
  // Tasa de cambio del cuaternión a partir del giroscopio
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
    
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + 
          _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + 
          my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + 
            _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;
    
    // Gradiente descendente
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - 
          _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + 
          (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + 
          _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 
          4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + 
          _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + 
          (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (0.5f - q1q1 - q2q2) - my) + 
          (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 
          4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + 
          (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + 
          (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (0.5f - q1q1 - q2q2) - my) + 
          (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + 
          (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + 
          (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (0.5f - q1q1 - q2q2) - my) + 
          _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    
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
  if (qNorm > 0.0001f) {
    recipNorm = 1.0f / qNorm;
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
  }
}

// ==================== CALIBRACIÓN DE SENSORES ====================
void calibrateSensors() {
  Serial.println("[CALIBRACIÓN] Estabilizando IMU...");
  delay(500);
  
  float sumPitch = 0, sumRoll = 0;
  
  for (int i = 0; i < 50; i++) {
    if (mpu.update()) {
      float ax = mpu.getAccX(), ay = mpu.getAccY(), az = mpu.getAccZ();
      float gx = mpu.getGyroX(), gy = mpu.getGyroY(), gz = mpu.getGyroZ();
      float mx = mpu.getMagX(), my = mpu.getMagY(), mz = mpu.getMagZ();
      
      updateOrientation(ax, ay, az, gx, gy, gz, mx, my, mz, 0.01f);
      
      sumPitch += asin(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / PI;
      sumRoll += atan2(2.0f * (q0 * q1 + q2 * q3), 
                       1.0f - 2.0f * (q1 * q1 + q2 * q3)) * 180.0f / PI;
    }
    delay(10);
  }
  
  pitchOffset = sumPitch / 50;
  rollOffset = sumRoll / 50;
  Serial.print("[CALIBRACIÓN] Offset Pitch: ");
  Serial.print(pitchOffset);
  Serial.print("°, Roll: ");
  Serial.println(rollOffset);
}

// ==================== IMPRIMIR MAC PARA VERIFICACIÓN ====================
void printMAC() {
  uint8_t mac[6];
  // Cambiamos ESP_MAC_WIFI_STA por ESP_MAC_WIFI_SOFTAP o simplemente usamos WiFi.macAddress()
  // Pero para mantener el estilo de bajo nivel, usamos la constante correcta:
  esp_read_mac(mac, ESP_MAC_WIFI_STA); 
  Serial.print("[MAC] Mi dirección MAC: ");
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║ PULSERA BIOCHRONORITMIC - EMISOR      ║");
  Serial.println("║ Modo: ESP-NOW Ultra Rápido            ║");
  Serial.println("╚════════════════════════════════════════╝");
  
  // Inicializar I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  
  i2cMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();
  
  // Inicializar sensores
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
    
    // MAX30102 (oxímetro)
    max_present = particleSensor.begin(Wire, I2C_SPEED_FAST);
    if (max_present) {
      particleSensor.setup(0x3F, 1, 2, 100, 411, 4096);
      Serial.println("[OK] MAX30102 detectado");
    } else {
      Serial.println("[ERROR] MAX30102 no detectado");
    }
    
    // BME280 (temperatura/presión)
    bme_present = bme.begin(0x76);
    if (bme_present) {
      Serial.println("[OK] BME280 detectado");
    } else {
      Serial.println("[ERROR] BME280 no detectado");
    }
    
    // MPU9250 (IMU)
    mpu_present = (mpu.setup(0x68) >= 0);
    if (mpu_present) {
      mpu.calibrateAccelGyro();
      Serial.println("[OK] MPU9250 detectado");
      calibrateSensors();
    } else {
      Serial.println("[ERROR] MPU9250 no detectado");
    }
    
    xSemaphoreGive(i2cMutex);
  }
  
  // ==================== CONFIGURACIÓN ESP-NOW ====================
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  // Mostrar mi MAC (útil para verificar)
  printMAC();
  
  // Configurar potencia máxima
  esp_wifi_set_max_tx_power(78);  // 19.5 dBm (máximo)
  esp_wifi_set_ps(WIFI_PS_NONE);   // Sin sleep
  
  // ¡CRÍTICO! Forzar canal 1 para coincidir con el robot
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init falló");
    while (1) delay(100);
  }
  
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);
  
  // Registrar peer (Robot)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, robotMAC, 6);
  peerInfo.channel = ESP_NOW_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ERROR] No se pudo añadir peer Robot");
  } else {
    Serial.print("[OK] Peer Robot registrado: ");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", robotMAC[0], robotMAC[1], robotMAC[2],
                  robotMAC[3], robotMAC[4], robotMAC[5]);
  }
  
  // Crear tareas en cores específicos
  xTaskCreatePinnedToCore(BiometricTask, "BiometricTask", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(IMUTask, "IMUTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(SendTask, "SendTask", 4096, NULL, 1, NULL, 1);
  
  Serial.print("[OK] Sistema listo - Transmitiendo en canal ");
  Serial.println(ESP_NOW_CHANNEL);
  Serial.println("[INFO] Esperando conexión con el Robot...\n");
}

// ==================== LOOP VACÍO (todo en tareas RTOS) ====================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
