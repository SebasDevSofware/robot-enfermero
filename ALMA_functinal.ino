#include <Wire.h>
// MAX30102 PPG Sensor
#include "MAX30105.h"
#include "spo2_algorithm.h"
// OLED 0.96
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

MAX30105 particleSensor;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

volatile int32_t g_spo2 = 0;
volatile int32_t g_heartRate = 0;
volatile int8_t g_validSPO2 = 0;
volatile int8_t g_validHeartRate = 0;
SemaphoreHandle_t i2cMutex;

// Ajuste de corriente para mejor penetración en muñeca (aprox 12mA)
byte ledBrightness = 0x24; // Referencia Tabla 8
// Aumentamos promedio a 8 para reducir ruido (SMP_AVE = 011)
byte sampleAverage = 8; // Referencia Tabla 3
byte ledMode = 2;       // Red + IR
// Usamos 100Hz real, valor estándar del datasheet
byte sampleRate = 100; // Referencia Tabla 6
// 411us da resolución de 18-bits (Máxima precisión)
int pulseWidth = 411; // Referencia Tabla 7
int adcRange = 4096;  // Rango estándar

void mostrarDatosOLED();

void TaskSalud(void *pvParameters)
{
  uint32_t irBuffer[100];
  uint32_t redBuffer[100];

  for (;;)
  {
    int samplesTaken = 0;

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
    {
      particleSensor.clearFIFO();
      xSemaphoreGive(i2cMutex);
    }

    while (samplesTaken < 100)
    {
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
      {
        particleSensor.check();

        while (particleSensor.available())
        {
          redBuffer[samplesTaken] = particleSensor.getRed();
          irBuffer[samplesTaken] = particleSensor.getIR();
          particleSensor.nextSample();

          samplesTaken++;
          if (samplesTaken >= 100)
            break;
        }
        xSemaphoreGive(i2cMutex);
      }

      vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    int32_t s, hr;
    int8_t vs, vhr;

    maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &s, &vs, &hr, &vhr);

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
    {
      if (irBuffer[99] < 50000)
      {
        g_heartRate = 0;
        g_spo2 = 0;
      }
      else
      {
        g_spo2 = s;
        g_heartRate = hr;
      }
      xSemaphoreGive(i2cMutex);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  i2cMutex = xSemaphoreCreateMutex();

  // ESP32 I2C Pines
  Wire.begin(21, 22);

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
  {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
      Serial.println("OLED Fallo");

    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println("CALIBRANDO SENSOR...");
    display.display();

    if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
      Serial.println("Error MAX30102");

    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

    particleSensor.setPulseAmplitudeGreen(0);

    xSemaphoreGive(i2cMutex);
  }

  xTaskCreatePinnedToCore(TaskSalud, "SaludTask", 10000, NULL, 1, NULL, 0);
}

void loop()
{
  if (xSemaphoreTake(i2cMutex, (TickType_t)50))
  {
    mostrarDatosOLED();

    Serial.printf("BPM: %d   SpO2: %d%%  \n", g_heartRate, g_spo2);

    xSemaphoreGive(i2cMutex);
  }

  delay(100);
}

void mostrarDatosOLED()
{
  display.clearDisplay();

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("ALMA");

  display.drawLine(0, 18, 128, 18, WHITE);

  display.setTextSize(1);
  display.setCursor(0, 25);

  if (g_heartRate == 0 || g_spo2 == 0)
  {
    display.println("Coloque la muneca...");
    display.setCursor(0, 40);
    display.print("Analizando...");
  }
  else
  {
    display.setTextSize(2);
    display.setCursor(0, 30);
    display.print("BPM: ");
    display.println(g_heartRate);

    display.setCursor(0, 50);
    display.setTextSize(1);
    display.print("SpO2: ");
    display.print(g_spo2);
    display.println("%");
  }

  display.display();
}