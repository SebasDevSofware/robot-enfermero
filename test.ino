#include "Arduino.h"
#include <WiFi.h>
#include "DFRobotDFPlayerMini.h"

// ==================== CONFIGURACIÓN PINES (Referencia: robot-enfermero.ino.cpp) =====================
#define BUZZER_PIN 18
#define LED_PIN 22

// Motor A (Izquierdo)
#define IN1_MOTORES 25
#define IN2_MOTORES 26
#define ENA_MOTORES 4

// Motor B (Derecho)
#define IN3_MOTORES 32
#define IN4_MOTORES 33
#define ENB_MOTORES 12

// Sensor Ultrasónico HC-SR04
#define TRIG_PIN 5 
#define ECHO_PIN 35 

// ==================== CONFIGURACIÓN MP3 (Referencia: codigo-mp3.ino) =====================
// Usamos UART2 del ESP32: Pin 16 (RX) y Pin 17 (TX)
HardwareSerial myHardwareSerial(2); 
DFRobotDFPlayerMini myDFPlayer;

// ==================== CONFIGURACIÓN WIFI ====================
const char* AP_SSID = "TEST_ROBOT_ENFERMERO";
const char* AP_PASS = "12345678";

// ==================== FUNCIONES DE APOYO ====================

float leerDistancia() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999.0f;
  
  float distance = duration * 0.034 / 2;
  return (distance > 400) ? 400 : distance;
}

void motoresDetener() {
  digitalWrite(IN1_MOTORES, LOW);
  digitalWrite(IN2_MOTORES, LOW);
  digitalWrite(IN3_MOTORES, LOW);
  digitalWrite(IN4_MOTORES, LOW);
  analogWrite(ENA_MOTORES, 0);
  analogWrite(ENB_MOTORES, 0);
}

void motoresAvanzar(int vel) {
  digitalWrite(IN1_MOTORES, HIGH);
  digitalWrite(IN2_MOTORES, LOW);
  digitalWrite(IN3_MOTORES, HIGH);
  digitalWrite(IN4_MOTORES, LOW);
  analogWrite(ENA_MOTORES, vel);
  analogWrite(ENB_MOTORES, vel);
}

// ==================== SETUP: SECUENCIA DE PRUEBA ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   TEST DE HARDWARE - ROBOT ENFERMERO   ║");
  Serial.println("╚════════════════════════════════════════╝");

  // 1. Inicializar Pines
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(IN1_MOTORES, OUTPUT);
  pinMode(IN2_MOTORES, OUTPUT);
  pinMode(ENA_MOTORES, OUTPUT);
  pinMode(IN3_MOTORES, OUTPUT);
  pinMode(IN4_MOTORES, OUTPUT);
  pinMode(ENB_MOTORES, OUTPUT);
  motoresDetener();

  // 2. Probar WiFi AP
  Serial.println("[1/6] Activando WiFi AP...");
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("      SSID: "); Serial.println(AP_SSID);
  Serial.print("      IP:   "); Serial.println(WiFi.softAPIP());
  delay(1000);

  // 3. Probar LED y Buzzer
  Serial.println("[2/6] Probando LED (1s)...");
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);

  Serial.println("[3/6] Probando Buzzer (1s)...");
  tone(BUZZER_PIN, 1000);
  delay(1000);
  noTone(BUZZER_PIN);

  Serial.println("      Probando LED + Buzzer simultáneos (1s)...");
  digitalWrite(LED_PIN, HIGH);
  tone(BUZZER_PIN, 1500);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  noTone(BUZZER_PIN);

  // 4. Probar Motores
  Serial.println("[4/6] Probando Motores individualmente...");
  Serial.println("      Motor A (Izquierdo) adelante...");
  digitalWrite(IN1_MOTORES, HIGH);
  digitalWrite(IN2_MOTORES, LOW);
  analogWrite(ENA_MOTORES, 150);
  delay(1000);
  motoresDetener();
  delay(500);

  Serial.println("      Motor B (Derecho) adelante...");
  digitalWrite(IN3_MOTORES, HIGH);
  digitalWrite(IN4_MOTORES, LOW);
  analogWrite(ENB_MOTORES, 150);
  delay(1000);
  motoresDetener();
  delay(500);

  Serial.println("      Ambos motores adelante...");
  motoresAvanzar(180);
  delay(1500);
  motoresDetener();

  // 5. Probar MP3
  Serial.println("[5/6] Inicializando comunicación MP3 (UART2)...");
  myHardwareSerial.begin(9600, SERIAL_8N1, 16, 17);
  if (myDFPlayer.begin(myHardwareSerial)) {
    Serial.println("      Módulo MP3 OK. Reproduciendo mp3/0003.mp3...");
    myDFPlayer.volume(25);
    myDFPlayer.playMp3Folder(3); 
  } else {
    Serial.println("      [ERROR] No se pudo conectar con el DFPlayer.");
  }

  // 6. Finalización
  Serial.println("[6/6] Iniciando monitoreo ultrasónico constante...");
  Serial.println("\n--- TEST INICIAL COMPLETADO ---");
  Serial.println("Observa el Monitor Serial para ver la distancia.\n");
}

// ==================== LOOP: MONITOREO CONSTANTE ====================
void loop() {
  float distancia = leerDistancia();
  
  Serial.print("Distancia detectada: ");
  Serial.print(distancia);
  Serial.println(" cm");

  // Feedback visual: Si hay algo a menos de 20cm, encender LED
  if (distancia < 20) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  // Monitor de estado del MP3
  if (myDFPlayer.available()) {
    uint8_t type = myDFPlayer.readType();
    int value = myDFPlayer.read();
    if (type == DFPlayerPlayFinished) {
       Serial.print("-> Notificación MP3: Archivo ");
       Serial.print(value);
       Serial.println(" terminado.");
    }
  }

  delay(300); // Frecuencia de muestreo
}
