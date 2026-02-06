#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU9250.h> // Librería de Hideaki Tai

// --- CONFIGURACIÓN OLED ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- OBJETO MPU ---
MPU9250 mpu;

// --- VARIABLES GLOBALES ---
float ax, ay, az;
float roll, pitch, yaw;
SemaphoreHandle_t i2cMutex;

// Tarea para leer el movimiento de forma independiente
void TaskMovimiento(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
      if (mpu.update()) {
        // Obtenemos aceleración
        ax = mpu.getAccX();
        ay = mpu.getAccY();
        az = mpu.getAccZ();
        
        // Obtenemos ángulos de inclinación (Roll y Pitch)
        roll  = mpu.getRoll();
        pitch = mpu.getPitch();
        yaw   = mpu.getYaw();
      }
      xSemaphoreGive(i2cMutex);
    }
    vTaskDelay(20 / portTICK_PERIOD_MS); // 50Hz es perfecto para detectar caídas
  }
}

void setup() {
  Serial.begin(115200);
  i2cMutex = xSemaphoreCreateMutex();
  Wire.begin(21, 22);

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    // Inicializar OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      Serial.println("OLED no encontrada");
    }

    // Inicializar MPU9250
    if (!mpu.setup(0x68)) { 
      Serial.println("MPU9250 no conectado. Revisa cables!");
      while (1); 
    }

    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println("MPU9250 LISTO");
    display.display();
    
    xSemaphoreGive(i2cMutex);
  }

  // Creamos la tarea en el Núcleo 1
  xTaskCreatePinnedToCore(TaskMovimiento, "MovTask", 5000, NULL, 2, NULL, 1);
}

void loop() {
  if (xSemaphoreTake(i2cMutex, (TickType_t)20)) {
    display.clearDisplay();
    
    // Título
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("ALMA - MOVIMIENTO");
    display.drawLine(0, 10, 128, 10, WHITE);

    // Mostrar Aceleración (G's)
    display.setCursor(0, 15);
    display.printf("Accel X: %.2f", ax);
    display.setCursor(0, 25);
    display.printf("Accel Y: %.2f", ay);
    display.setCursor(0, 35);
    display.printf("Accel Z: %.2f", az);

    // Mostrar Inclinación (Grados)
    display.setCursor(0, 50);
    display.printf("P:%.1f R:%.1f", pitch, roll);

    // Dibujar un pequeño horizonte artificial (opcional)
    int horizonte = map(pitch, -90, 90, 64, 10);
    display.drawLine(80, horizonte, 120, horizonte, WHITE);

    display.display();
    xSemaphoreGive(i2cMutex);
  }
  delay(100); // Refresco de pantalla
}