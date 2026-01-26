#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_MLX90614.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// --- CONFIGURACIÓN OLED ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- OBJETOS Y SENSORES ---
MAX30105 particleSensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BME280 bme;

// --- SEMÁFORO (MUTEX) ---
SemaphoreHandle_t i2cMutex;

// --- VARIABLES GLOBALES ---
volatile int32_t g_spo2 = 0;
volatile int32_t g_heartRate = 0;
volatile int8_t g_validSPO2 = 0;
volatile int8_t g_validHeartRate = 0;
volatile float g_tempCorporal = 0;
volatile float g_humedad = 0;
volatile float g_presion = 0;
String g_estadoGlobal = "INICIANDO";
unsigned long tiempoImpacto = 0;
bool posibleCaida = false;

void mostrarDatosOLED();
void filtroCaidas(float mag, float inclinacion);

// --- TAREA DE SALUD (NÚCLEO 0) ---
void TaskSalud(void *pvParameters)
{
  uint32_t irBuffer[100];
  uint32_t redBuffer[100];

  for (;;)
  {
    // MAX30102
    for (byte i = 0; i < 100; i++)
    {
      // Pedimos permiso para usar I2C
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
      {
        while (particleSensor.available() == false)
          particleSensor.check();
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample();
        xSemaphoreGive(i2cMutex);
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    int32_t s, hr;
    int8_t vs, vhr;
    maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &s, &vs, &hr, &vhr);

    // Calcular Biometría y Temperatura
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
    {
      g_spo2 = s;
      g_validSPO2 = vs;
      g_heartRate = hr;
      g_validHeartRate = vhr;
      g_tempCorporal = mlx.readObjectTempC();

      static int contadorLectura = 0;
      contadorLectura++;
      if (contadorLectura >= 10)
      {
        g_presion = bme.readPressure() / 100.0F;
        g_humedad = bme.readHumidity();
        contadorLectura = 0;
      }

      xSemaphoreGive(i2cMutex);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  i2cMutex = xSemaphoreCreateMutex();
  Wire.begin(21, 22); // Pines estándar SDA/SCL

  // Variables del MAX30102
  byte ledBrightness = 40;
  byte sampleAverage = 4;
  byte ledMode = 2;
  byte sampleRate = 70;
  int pulseWidth = 411;
  int adcRange = 4096;

  // Inicializar sensores con protección Mutex

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
  {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
      Serial.println("OLED Fallo");
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println("INICIANDO PULSERA...");
    display.display();

    if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
      Serial.println("Error MAX30102");
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    if (!mlx.begin())
      Serial.println("Error MLX90614");
    if (!bno.begin())
      Serial.println("Error BNO055");
    if (!bme.begin(0x76))
      Serial.println("Error BME280"); // Error de sintaxis corregido

    bno.setExtCrystalUse(true);
    xSemaphoreGive(i2cMutex);
  }

  xTaskCreatePinnedToCore(TaskSalud, "SaludTask", 10000, NULL, 1, NULL, 0);
}

void loop()
{
  // --- NÚCLEO 1: MOVIMIENTO Y CAÍDAS (Corre a alta velocidad) ---
  sensors_event_t eventAcc, eventOri;

  if (xSemaphoreTake(i2cMutex, (TickType_t)5))
  { // Intenta tomar el I2C por 10ms
    bno.getEvent(&eventAcc, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&eventOri, Adafruit_BNO055::VECTOR_EULER);

    static unsigned long lastOLED = 0;
    if (millis() - lastOLED > 300)
    {
      mostrarDatosOLED();
      lastOLED = millis();
    }

    xSemaphoreGive(i2cMutex);
  }

  float mag = sqrt(pow(eventAcc.acceleration.x, 2) + pow(eventAcc.acceleration.y, 2) + pow(eventAcc.acceleration.z, 2));
  float inclinacion = eventOri.orientation.y;

  if (mag > 40.0 && !posibleCaida)
  {
    posibleCaida = true;
    tiempoImpacto = millis();
    Serial.println("!!! IMPACTO DETECTADO !!!");
  }

  filtroCaidas(mag, inclinacion);

  // Monitor serial
  // static unsigned long lastPrint = 0;
  // if (millis() - lastPrint > 500) {
  //   Serial.printf("\n[SALUD] HR: %d | SpO2: %d%% | Temp: %.1fC\n", g_heartRate, g_spo2, g_tempCorporal);
  //   Serial.printf("[MOV] Mag: %.2f | Estado: %s\n", mag, g_estadoGlobal.c_str());
  //   lastPrint = millis();
  // }

  delay(50);
}

void mostrarDatosOLED()
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("WRO 2026 - SALUD");
  display.drawLine(0, 10, 128, 10, WHITE);

  display.setCursor(0, 15);
  display.printf("BPM: %d   SpO2: %d%%", g_heartRate, g_spo2);
  display.setCursor(0, 28);
  display.printf("Temp: %.1f C", g_tempCorporal);

  display.setCursor(0, 45);
  display.println("ESTADO:");
  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print(g_estadoGlobal);

  display.display();
}

void filtroCaidas(float mag, float inclinacion)
{
  if (posibleCaida)
  {
    if (millis() - tiempoImpacto > 3000)
    {
      if (mag > 8.5 && mag < 11.5)
      {
        if (abs(inclinacion) > 70)
        {
          g_estadoGlobal = "CAIDA CONFIRMADA";
        }
        else
        {
          g_estadoGlobal = "RECUPERADO";
          posibleCaida = false;
        }
      }
      else
      {
        g_estadoGlobal = "MOVIMIENTO";
        posibleCaida = false;
      }
    }
    else
    {
      g_estadoGlobal = "ANALIZANDO...";
    }
  }
  else
  {
    if (mag > 12.0 && mag < 25.0)
      g_estadoGlobal = "CAMINANDO";
    else if (mag > 25.0)
      g_estadoGlobal = "CORRIENDO";
    else if (abs(inclinacion) > 70)
      g_estadoGlobal = "ACOSTADO";
    else
      g_estadoGlobal = "ESTABLE";
  }
}
