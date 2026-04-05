#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ==================== UMBRALES MÉDICOS ====================
#define ZONA_PELIGRO_HR_MIN 50
#define ZONA_PELIGRO_HR_MAX 120
#define ZONA_CRITICA_HR_MIN 40
#define ZONA_CRITICA_HR_MAX 150

#define ZONA_PELIGRO_SPO2 92
#define ZONA_CRITICA_SPO2 88

#define UMBRAL_FIEBRE 38.0
#define UMBRAL_HIPOTERMIA 35.0

// La misma estructura exacta del emisor (Intocable)
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

// Variables globales para almacenar el resultado de la IA en el receptor
uint8_t nivelRiesgoIA = 0;
char diagnosticoIA[128] = "";
char recomendacionIA[128] = "";

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
// Adaptado para leer desde 'incomingData' sin usar Mutex, 
// ya que se llamará de forma segura en el loop().
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

  if (nivelRiesgoIA >= 2) {
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

// Callback de ESP-NOW: Se ejecuta en contexto de interrupción, debe ser rápido.
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
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
  Serial.print("MI MAC (Cópiala en el código de la pulsera): ");
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
    // 1. Procesar los datos de manera secuencial y segura fuera de la interrupción
    analizarContextoMedico();
    sistemaExpertoIA();
    
    // 2. Reporte en consola
    Serial.println("\n=== ANALISIS CLINICO ALMA ===");
    Serial.printf("BIO -> BPM: %d | SpO2: %d%% | Señal: %d\n", 
                  incomingData.heartRate, incomingData.spo2, incomingData.signalQuality);
    
    Serial.printf("ENV -> Temp Amb: %.1f C | Presion: %.1f hPa\n", 
                  incomingData.temperature, incomingData.pressure);
                  
    Serial.printf("CTX -> ESTADO RAPIDO: %s\n", incomingData.estadoGlobal);
      
    if (incomingData.fallDetected) Serial.println("!!! ALERTA DE IMPACTO DETECTADA !!!");

    Serial.println("\n--- REPORTE PROFUNDO: LOGICA DIFUSA ---");
    Serial.printf("Nivel de Riesgo (0-3): %d\n", nivelRiesgoIA);
    Serial.printf("Diagnóstico: %s\n", diagnosticoIA);
    Serial.printf("Recomendación: %s\n", recomendacionIA);
    
    // 3. Lógica de actuación basada en los resultados de la IA
    // Ideal para enlazar con los periféricos de ALMA (Motores, Audio, Dispensador)
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
    
    // Resetear bandera para esperar el próximo paquete
    mensaje_recibido = false; 
  }
}