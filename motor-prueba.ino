// --- DEFINICIÓN DE PINES ---

// Motor A (Izquierdo)
#define IN1_MOTORES 25   
#define IN2_MOTORES 26   
#define ENA_MOTORES 4    // PWM_A

// Motor B (Derecho)
#define IN3_MOTORES 32   
#define IN4_MOTORES 33   
#define ENB_MOTORES 2    // PWM_B

// Propiedades del PWM
const int frecuencia = 5000;
const int resolucion = 8; // 0-255

void setup() {
  Serial.begin(115200);

  // Configurar pines de dirección
  pinMode(IN1_MOTORES, OUTPUT);
  pinMode(IN2_MOTORES, OUTPUT);
  pinMode(IN3_MOTORES, OUTPUT);
  pinMode(IN4_MOTORES, OUTPUT);

  // NUEVA API DE ESP32 (Core 3.x)
  // No se necesita ledcSetup. ledcAttach configura todo automáticamente.
  ledcAttach(ENA_MOTORES, frecuencia, resolucion);
  ledcAttach(ENB_MOTORES, frecuencia, resolucion);

  Serial.println("Motores configurados con API nueva.");
}

void detener() {
  digitalWrite(IN1_MOTORES, LOW);
  digitalWrite(IN2_MOTORES, LOW);
  digitalWrite(IN3_MOTORES, LOW);
  digitalWrite(IN4_MOTORES, LOW);
  ledcWrite(ENA_MOTORES, 0);
  ledcWrite(ENB_MOTORES, 0);
}

void moverAdelante(int velocidad) {
  digitalWrite(IN1_MOTORES, HIGH);
  digitalWrite(IN2_MOTORES, LOW);
  digitalWrite(IN3_MOTORES, HIGH);
  digitalWrite(IN4_MOTORES, LOW);
  ledcWrite(ENA_MOTORES, velocidad);
  ledcWrite(ENB_MOTORES, velocidad);
}

void moverAtras(int velocidad) {
  digitalWrite(IN1_MOTORES, LOW);
  digitalWrite(IN2_MOTORES, HIGH);
  digitalWrite(IN3_MOTORES, LOW);
  digitalWrite(IN4_MOTORES, HIGH);
  ledcWrite(ENA_MOTORES, velocidad);
  ledcWrite(ENB_MOTORES, velocidad);
}

void loop() {
  Serial.println("Hacia adelante...");
  moverAdelante(200); 
  delay(1500);

  detener();
  delay(500);

  Serial.println("Hacia atras...");
  moverAtras(200);
  delay(1500);

  Serial.println("Pausa de 1 segundo...");
  detener();
  delay(1000);
}