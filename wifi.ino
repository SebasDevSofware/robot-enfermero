#include <Wifi.h>

const char *ssid = "SSID_NAME";
const char *password = "SSID_PASSWORD";

volatile bool isConnected = false;
int connectionRetries = 0;
const int MAX_RETRIES = 5;
unsigned long lastCheck = 0;
const unsigned long interval = 15000;

void WiFiEvent(WiFiEvent_t event)
{
    switch (event)
    {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        isConnected = true;
        connectionRetries = 0;
        Serial.printf("[IP]: %s\n", WiFi.localIP().toString().c_str());
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        isConnected = false;
        break;
    default:
        break;
    }
}

void setup()
{
    Serial.begin(115200);

    // WiFi Setup
    // Set Wif Connection in Station Mode (Client Mode)
    WiFi.mode(WIFI_STA);
    // Set WiFi Event Handler
    WiFi.onEvent(WiFiEvent);
    // Sleep Modem Disabled for Max Performance and Velocity
    WiFi.setSleep(false);
    // Start WiFi Connection
    WiFi.begin(ssid, password);
}

void loop()
{
    // Main Logic
    // Read Sensors Data and Proces Them

    if (millis() - lastCheck > interval)
    {
        monitorNetworkHealth();
        lastCheck = millis();
    }

    delayMicroseconds(3000);
}

// Check Connection Health for no losed packets
void monitorNetworkHealth()
{
    if (isConnected)
    {
        int8_t rssi = WiFi.RSSI();
        if (rssi < -80)
        {
            // Turn On LED or Buzzer to Warn Low Signal
        }
    }
    else
    {
        if (connectionRetries < MAX_RETRIES)
        {
            WiFi.begin(ssid, password);
            connectionRetries++;
        }
        else
        {
            ESP.restart();
        }
    }
}