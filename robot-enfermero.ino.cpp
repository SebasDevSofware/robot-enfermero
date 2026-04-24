#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

// ==================== CONFIGURACIÓN DE RED LOCAL ====================
#define AP_SSID "BIOCHRONORITMIC_IA_CENTER"
#define AP_PASS "12345678"

// ==================== CONFIGURACIÓN PINES =====================
#define BUZZER_PIN 18
#define LED_PIN 22
// Motor A (Izquierdo)
#define IN1_MOTORES 27   // IN_A (Dirección)
#define IN2_MOTORES 14   // IN_B (Dirección)
#define ENA_MOTORES 13    // PWM_A (Velocidad)
// Motor B (Derecho)
#define IN3_MOTORES 32   // IN_A (Dirección)
#define IN4_MOTORES 33   // IN_B (Dirección)
#define ENB_MOTORES 2    // PWM_B (Velocidad)
// Sensor Ultrasónico HC-SR04
#define TRIG_PIN 17
#define ECHO_PIN 16

// ==================== UMBRALES DINÁMICOS ====================
#define HR_NORMAL_MIN 60
#define HR_NORMAL_MAX 100
#define HR_EJERCICIO_MAX 150

#define SPO2_NORMAL_MIN 96
#define SPO2_ALERTA 92
#define SPO2_CRITICO 88

#define DISTANCIA_SEGURIDAD_CM 30
#define DISTANCIA_CRITICA_CM 10

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
bool killSwitchActive = false;

// ==================== TAREA DE SEGURIDAD (CORE 0) ====================
void TaskSafety(void *pvParameters) {
  while (;;) {
    // Monitoreo constante del ultrasónico cada 20ms para máxima seguridad
    // Frenado inmediato si detecta obstáculo crítico 
    // Reducido a 20ms para vigilancia constante
  }
}

// ==================== TAREA DE ALERTAS (CORE 1) ====================
void TaskAlerts(void *pvParameters) {
  while (;;) {
      // LED parpadeante durante alertas para llamar la atención
      // Buzzer activandonse durante alertas
      // LED SIEMPRE ENCENDIDO en estado normal para indicar sistema operativo
  }
}

// ==================== SENSOR ULTRASÓNICO (HC-SR04) ====================
float leerDistanciaUltrasonico() {
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
  digitalWrite(ENA_MOTORES, 0);
  digitalWrite(ENB_MOTORES, 0);
}

void motoresDarVueltas(int velocidad) {
}

// ==================== CÁLCULO DE ALTITU ====================
float calcularAltitud(float presionHpa) {
}

// ==================== FUZZY LOGIC IA ====================
void sistemaExpertoIA() {
  // 1. Actualizar umbrales según altitud
  actualizarUmbralesDinamicos();

  // 2. Actualizar búferes temporales
  if (processedData.heartRate > 0) bufferPush(&bufferHR, (float)processedData.heartRate);
  if (processedData.spo2 > 0) bufferPush(&bufferSpO2, (float)processedData.spo2);
  bufferPush(&bufferTemp, processedData.temperature);

  // 3. Calcular derivadas
  derivadaHR = bufferGetTendencia(&bufferHR);
  derivadaSpO2 = bufferGetTendencia(&bufferSpO2);
  derivadaTemp = bufferGetTendencia(&bufferTemp);

  // 4. Actualizar tendencia visual
  if (derivadaHR > 3) strcpy(tendenciaHR, "SUBIENDO");
  else if (derivadaHR < -3) strcpy(tendenciaHR, "BAJANDO");
  else strcpy(tendenciaHR, "ESTABLE");

  if (derivadaSpO2 < -1) strcpy(tendenciaSpO2, "CAIDA");
  else if (derivadaSpO2 > 1) strcpy(tendenciaSpO2, "SUBIENDO");
  else strcpy(tendenciaSpO2, "ESTABLE");

  // 5. Clasificar actividad
  strcpy(tipoActividad, clasificarActividad());

  // === CÁLCULO DE RIESGO CON FUSIÓN SENSORIAL ===
  float riesgoHR = 0.0f, riesgoSpO2 = 0.0f, riesgoTemp = 0.0f, riesgoMov = 0.0f;
  float riesgoDerivada = 0.0f;
  float riesgoTotal = 0.0f;

  // --- RIESGO CARDÍACO (con fusión sensorial) ---
  if (processedData.heartRate > 0) {
    bool esFalsaAlarma = esFalsaAlarmaTaquicardia();

    if (processedData.heartRate > 120 && !esFalsaAlarma) {
      riesgoHR = 0.7f + (processedData.heartRate - 120) / 50.0f;
      if (riesgoHR > 1.0f) riesgoHR = 1.0f;
      estadoPaciente = ESTADO_TAQUICARDIA_REPOSO;
    } else if (processedData.heartRate < 50 && processedData.heartRate > 0) {
      riesgoHR = 0.6f + (50 - processedData.heartRate) / 30.0f;
      if (riesgoHR > 1.0f) riesgoHR = 1.0f;
      estadoPaciente = ESTADO_BRADICARDIA;
    } else if (esFalsaAlarma) {
      // Reducir riesgo si es ejercicio válido
      riesgoHR *= 0.3f;
      estadoPaciente = ESTADO_ACTIVIDAD_FISICA;
    }

    // Detectar arritmia por variabilidad extrema
    if (abs(derivadaHR) > 15) {
      riesgoHR += 0.2f;
    }
  }

  // --- RIESGO RESPIRATORIO (con umbrales dinámicos) ---
  if (processedData.spo2 > 0) {
    if (processedData.spo2 <= umbralSpO2Dinamico - 10) {
      riesgoSpO2 = 1.0f;
      estadoPaciente = ESTADO_HIPOXIA;
    } else if (processedData.spo2 <= umbralSpO2Dinamico) {
      riesgoSpO2 = 0.7f;
    } else if (processedData.spo2 <= umbralSpO2Dinamico + 3) {
      riesgoSpO2 = 0.3f;
    }

    // DETECCIÓN TEMPRANA: SpO2 cayendo rápidamente
    if (derivadaSpO2 < -2.0f) {
      riesgoDerivada += 0.4f;
      strcat(diagnosticoIA, "DESATURACIÓN RÁPIDA. ");
    }
  }

  // --- RIESGO TÉRMICO ---
  if (processedData.temperature > 0) {
    if (processedData.temperature >= TEMP_GOLPE_CALOR) {
      riesgoTemp = 1.0f;
      estadoPaciente = ESTADO_GOLPE_CALOR;
    } else if (processedData.temperature >= TEMP_FIEBRE) {
      riesgoTemp = 0.5f + (processedData.temperature - TEMP_FIEBRE) / 4.0f;
      if (riesgoTemp > 1.0f) riesgoTemp = 1.0f;
    } else if (processedData.temperature <= TEMP_HIPOTERMIA) {
      riesgoTemp = 0.8f + (TEMP_HIPOTERMIA - processedData.temperature) / 5.0f;
      if (riesgoTemp > 1.0f) riesgoTemp = 1.0f;
    }
  }

  // --- RIESGO DE MOVIMIENTO/CAÍDA ---
  if (processedData.fallDetected) {
    riesgoMov = 1.0f;
    estadoPaciente = ESTADO_CAIDA;
  } else if (processedData.magnitude > UMBRAL_MAGNITUD_ALTA) {
    riesgoMov = 0.4f;
  }

  // === CÁLCULO DE TEMPERATURA CORPORAL ESTIMADA (FUZZY) ===
  temperatutraAproxIA = estimarTemperaturaFuzzy();

  // === CÁLCULO DE RIESGO TOTAL (PONDERADO) ===
  riesgoTotal = (riesgoHR * 0.30f) + (riesgoSpO2 * 0.35f) +
                (riesgoTemp * 0.15f) + (riesgoMov * 0.10f) +
                (riesgoDerivada * 0.10f);
  if (riesgoTotal > 1.0f) riesgoTotal = 1.0f;

  // === DETERMINAR NIVEL DE RIESGO ===
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

  // === GENERAR DIAGNÓSTICO Y RECOMENDACIÓN ===
  memset(diagnosticoIA, 0, sizeof(diagnosticoIA));
  memset(recomendacionIA, 0, sizeof(recomendacionIA));

  int patronSugerido = PATRON_NORMAL;

  if (nivelRiesgoIA >= 2 || processedData.fallDetected) {
    if (riesgoHR > 0.7f) {
      if (processedData.heartRate < 50) {
        strcat(diagnosticoIA, "BRADICARDIA SEVERA. ");
        patronSugerido = PATRON_BRADICARDIA;
      } else {
        strcat(diagnosticoIA, "TAQUICARDIA SEVERA. ");
        patronSugerido = PATRON_TAQUICARDIA;
      }
      strcat(recomendacionIA, "EVALUACIÓN CARDIOLÓGICA URGENTE. ");
    }
    if (riesgoSpO2 > 0.7f) {
      strcat(diagnosticoIA, "HIPOXEMIA CRÍTICA. ");
      strcat(recomendacionIA, "ADMINISTRAR OXÍGENO SUPLEMENTARIO. ");
      patronSugerido = PATRON_HIPOXIA;
    }
    if (riesgoTemp > 0.7f) {
      if (processedData.temperature >= TEMP_GOLPE_CALOR) {
        strcat(diagnosticoIA, "GOLPE DE CALOR. ");
        patronSugerido = PATRON_GOLPE_CALOR;
      } else if (processedData.temperature >= TEMP_FIEBRE) {
        strcat(diagnosticoIA, "HIPERTERMIA. ");
      } else {
        strcat(diagnosticoIA, "HIPOTERMIA. ");
      }
      strcat(recomendacionIA, "CONTROL TÉRMICO INMEDIATO. ");
    }
    if (processedData.fallDetected) {
      strcat(diagnosticoIA, "CAÍDA DETECTADA. ");
      strcat(recomendacionIA, "VERIFICAR ESTADO DE CONCIENCIA. ACTIVAR PROTOCOLO DE ASISTENCIA. ");
      patronSugerido = PATRON_CAIDA;
    }

    alertaDeEmergencia(15);
    patronSugerido = (patronSugerido == PATRON_NORMAL) ? PATRON_CRITICO : patronSugerido;
    xQueueSend(alertQueue, &patronSugerido, 0);

  } else if (nivelRiesgoIA == 1) {
    if (riesgoHR > 0.2f) strcat(diagnosticoIA, "ALTERACIÓN CARDÍACA LEVE. ");
    if (riesgoSpO2 > 0.2f) strcat(diagnosticoIA, "DESATURACIÓN LEVE. ");
    if (riesgoTemp > 0.2f) strcat(diagnosticoIA, "TEMPERATURA ANORMAL. ");
    strcat(recomendacionIA, "MONITOREO CONTINUO. ");
    patronAlertaActual = PATRON_MONITOREO;
  } else {
    strcat(diagnosticoIA, "PARÁMETROS NORMALES.");
    strcat(recomendacionIA, "CONTINÚE MONITOREO.");
    patronAlertaActual = PATRON_NORMAL;
  }

  strncpy(processedData.estadoGlobal, ESTADO_STR[estadoPaciente], 19);
  processedData.estadoGlobal[19] = '\0';
}

// Callback de ESP-NOW
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  static unsigned long lastRecvLog = 0;
  
  if (len == sizeof(TelemetryPacket)) {
    TelemetryPacket packet;
  } else {
    Serial.printf("[WARNING] Paquete descartado. Tamaño: %d (Esperado: %d)\n",
                   len, sizeof(TelemetryPacket));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n[SYSTEM] Iniciando BIOCHRONORITMIC...");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(IN1_MOTORES, OUTPUT);
  pinMode(IN2_MOTORES, OUTPUT);
  pinMode(ENA_MOTORES, OUTPUT);
  pinMode(IN3_MOTORES, OUTPUT);
  pinMode(IN4_MOTORES, OUTPUT);
  pinMode(ENB_MOTORES, OUTPUT);
  digitalWrite(IN1_MOTORES, LOW);
  digitalWrite(IN2_MOTORES, LOW);
  digitalWrite(IN3_MOTORES, LOW);
  digitalWrite(IN4_MOTORES, LOW);
  analogWrite(ENA_MOTORES, 0);
  analogWrite(ENB_MOTORES, 0);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  delay(500);  
  digitalWrite(BUZZER_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
  
  WiFi.mode(WIFI_AP);
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  
  if(WiFi.softAP(AP_SSID, AP_PASS, 1)) {
    Serial.println("[WIFI] AP Command Center iniciado en Canal 1.");
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW falló");
  } else {
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("[SYSTEM] ESP-NOW configurado.");
  }
  
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.addHandler(&events);
  server.begin();
  
  alertQueue = xQueueCreate(5, sizeof(int));
  
  xTaskCreatePinnedToCore(TaskSafety, "SafetyTask", 4096, NULL, 3, NULL, 0); 
  xTaskCreatePinnedToCore(TaskAlerts, "AlertTask", 4096, NULL, 1, NULL, 1);
  
  Serial.println("[SYSTEM] BIOCHRONORITMIC ONLINE.");
}

void loop() {
  if (mensaje_recibido) {
    analizarLoteData();
    sistemaExpertoIAElite();

    StaticJsonDocument<1024> doc;
    doc["heartRate"] = processedData.heartRate;
    doc["spo2"] = processedData.spo2;
    doc["temperature"] = processedData.temperature;
    doc["pressure"] = processedData.pressure;
    doc["roll"] = processedData.roll;
    doc["pitch"] = processedData.pitch;
    doc["yaw"] = processedData.yaw;
    doc["magnitude"] = processedData.magnitude;
    doc["estadoGlobal"] = processedData.estadoGlobal;
    doc["fallDetected"] = (int)processedData.fallDetected;
    doc["signalQuality"] = (int)processedData.signalQuality;
    doc["nivelRiesgoIA"] = nivelRiesgoIA;
    doc["diagnosticoIA"] = diagnosticoIA;
    doc["recomendacionIA"] = recomendacionIA;
    doc["temperatutraAproxIA"] = temperatutraAproxIA;
    doc["tipoActividad"] = tipoActividad;
    doc["altitudEstimada"] = altitudEstimada;
    doc["distanciaObstaculo"] = distanciaObstaculo;
    doc["killSwitchActive"] = killSwitchActive ? 1 : 0;
    doc["tendenciaHR"] = tendenciaHR;
    doc["tendenciaSpO2"] = tendenciaSpO2;

    String jsonResponse;
    serializeJson(doc, jsonResponse);
    events.send(jsonResponse.c_str(), "ia", millis());

    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.printf("║ ESTADO: %-30s║\n", ESTADO_STR[estadoPaciente]);
    Serial.printf("║ RIESGO: NIVEL %d                        ║\n", nivelRiesgoIA);
    Serial.println("╠════════════════════════════════════════╣");
    Serial.printf("║ BPM: %4d | SpO2: %3d%% | Temp: %5.1f°C ║\n",
                   processedData.heartRate, processedData.spo2, temperatutraAproxIA);
    Serial.printf("║ TENDENCIA HR: %-10s SpO2: %-10s║\n", tendenciaHR, tendenciaSpO2);
    Serial.printf("║ ACTIVIDAD: %-28s║\n", tipoActividad);
    Serial.printf("║ ALTITUD: %-6.0fm | SpO2 UMBrAL: %2.0f%%    ║\n",
                   altitudEstimada, umbralSpO2Dinamico);
    Serial.println("╠════════════════════════════════════════╣");
    Serial.printf("║ DIAG: %-33s║\n", diagnosticoIA);
    Serial.printf("║ RECO: %-33s║\n", recomendacionIA);
    Serial.println("╠════════════════════════════════════════╣");
    Serial.printf("║ OBSTÁCULO: %5.1fcm | KILL: %-8s║\n",
                   distanciaObstaculo, killSwitchActive ? "ACTIVO" : "INACTIVO");
    Serial.println("╚════════════════════════════════════════╝");

    mensaje_recibido = false;
  }


  vTaskDelay(pdMS_TO_TICKS(10)); 
}
