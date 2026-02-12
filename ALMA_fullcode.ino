//
// ******************************************************
// *                                                    *
// *  ALMA - Asistente Logístico de Monitoreo Avanzado  *
// *                                                    *
// ******************************************************
//

#include <Wire.h>
// MAX30102 PPG Sensor
#include "MAX30105.h"
#include "spo2_algorithm.h"
// OLED 0.96
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// MLX90614 IR Thermometer
#include <Adafruit_MLX90614.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "MPU9250.h"

// OLED Display variables
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Sensors Instances
MAX30105 particleSensor;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MPU9250 mpu;
Adafruit_BME280 bme;

// MUTEX
SemaphoreHandle_t i2cMutex;

// Global Varibales
volatile int32_t g_spo2 = 0;
volatile int32_t g_heartRate = 0;
volatile int8_t g_validSPO2 = 0;
volatile int8_t g_validHeartRate = 0;

// MAX30102 Vectors Configuration
byte ledBrightness = 0x24;
byte sampleAverage = 8; // Aumentamos promedio a 8 para reducir ruido (SMP_AVE = 011)
byte ledMode = 2;       // Red + IR
byte sampleRate = 100;  // Usamos 100Hz real, valor estándar del datasheet
int pulseWidth = 411;   // 411us da resolución de 18-bits (Máxima precisión)
int adcRange = 4096;

// volatile float g_tempCorporal = 0;
volatile float g_humedad = 0;
volatile float g_presion = 0;
String g_estadoGlobal = "INICIANDO";
unsigned long tiempoImpacto = 0;
bool detectedFall = false;

void mostrarDatosOLED();
void filtroCaidas(float mag, float inclination);

// CORE 0: Biometric Task
void BiometricTask(void *pvParameters)
{
    uint32_t irBuffer[100];
    uint32_t redBuffer[100];

    for (;;)
    {
        uint32_t samplesTaken = 0; // Inicializar contador de muestras
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
        {
            particleSensor.clearFIFO();
            xSemaphoreGive(i2cMutex);
        }

        // MAX30102 Read
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

        // Get Temperature, Pressure and Humidity
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

            // g_tempCorporal = mlx.readObjectTempC();

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
    Wire.begin(21, 22); // SDA/SCL

    // Sensors Initialization with MUTEX
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
        // if (!mlx.begin())
        // Serial.println("Error MLX90614");
        if (!bme.begin(0x76))
            Serial.println("Error BME280");
        if (!mpu.setup(0x68) < 0)
            Serial.println("Error MPU9250");
        else
            mpu.calibrateAccelGyro();

        xSemaphoreGive(i2cMutex);
    }

    xTaskCreatePinnedToCore(BiometricTask, "BiometricTask", 10000, NULL, 1, NULL, 0);
}

void loop()
{
    // CORE 1: MPU and Fall Detection
    float ax = 0, ay = 0, az = 0;
    float mag = 0;
    float inclination = 0;

    if (xSemaphoreTake(i2cMutex, (TickType_t)5))
    {
        if (mpu.update())
        {                       // Cambiado de readSensor
            ax = mpu.getAccX(); // Cambiado: quita el _mss
            ay = mpu.getAccY();
            az = mpu.getAccZ();
        }

        static unsigned long lastOLED = 0;
        if (millis() - lastOLED > 300)
        {
            mostrarDatosOLED();
            lastOLED = millis();
        }

        xSemaphoreGive(i2cMutex);
    }

    mag = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
    inclination = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) * 180.0 / PI;

    if (mag > 40.0 && !detectedFall)
    {
        detectedFall = true;
        tiempoImpacto = millis();
        Serial.println("!!! IMPACTO DETECTADO !!!");
    }

    filtroCaidas(mag, inclination);

    if (xSemaphoreTake(i2cMutex, (TickType_t)50))
    {
        mostrarDatosOLED();

        // Serial.printf("BPM: %d   SpO2: %d%%  \n", g_heartRate, g_spo2);

        xSemaphoreGive(i2cMutex);
    }

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
    // display.setCursor(0, 28);
    // display.printf("Temp: %.1f C", g_tempCorporal);

    display.setCursor(0, 45);
    display.println("ESTADO:");
    display.setTextSize(1);
    display.setCursor(0, 55);
    display.print(g_estadoGlobal);

    display.display();
}

void filtroCaidas(float mag, float inclination)
{
    if (detectedFall)
    {
        if (millis() - tiempoImpacto > 3000)
        {
            if (mag > 8.5 && mag < 11.5)
            {
                if (abs(inclination) > 70)
                {
                    g_estadoGlobal = "CAIDA CONFIRMADA";
                }
                else
                {
                    g_estadoGlobal = "RECUPERADO";
                    detectedFall = false;
                }
            }
            else
            {
                g_estadoGlobal = "MOVIMIENTO";
                detectedFall = false;
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
        else if (abs(inclination) > 70)
            g_estadoGlobal = "ACOSTADO";
        else
            g_estadoGlobal = "ESTABLE";
    }
}