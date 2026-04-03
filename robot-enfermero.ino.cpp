#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// La misma estructura exacta del emisor
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

TelemetryPacket incomingData;
volatile bool mensaje_recibido = false;

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  // Verificación analítica de integridad: ¿El tamaño del paquete coincide?
  if (len == sizeof(TelemetryPacket)) {
    memcpy(&incomingData, data, sizeof(incomingData));
    mensaje_recibido = true;
  } else {
    Serial.printf("[WARNING] Paquete descartado. Tamaño anómalo: %d bytes (Esperados: %d)\n", len, sizeof(TelemetryPacket));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  
  // Forzar Canal 1
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  delay(100);
  Serial.println("=========================================");
  Serial.print("MI MAC (Cópiala en el código de ALMA): ");
  Serial.println(WiFi.macAddress());
  Serial.println("=========================================");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Receptor de Telemetría Médico-Logística listo...");
}

void loop() {
  if (mensaje_recibido) {
    Serial.println("\n--- Nuevo Paquete de ALMA ---");
    Serial.printf("Tiempo: %lu ms\n", incomingData.timestamp);
    Serial.printf("BPM: %d | SpO2: %d%% | Señal Óptima: %s\n", 
                  incomingData.heartRate, 
                  incomingData.spo2, 
                  incomingData.signalQuality ? "SÍ" : "NO");
    Serial.printf("Estado: %s | Caída: %s\n", 
                  incomingData.estadoGlobal, 
                  incomingData.fallDetected ? "¡ALERTA!" : "Negativo");
    Serial.printf("Temp: %.2f °C | Presión: %.2f hPa\n", 
                  incomingData.temperature, 
                  incomingData.pressure);
    
    mensaje_recibido = false; 
  }
}