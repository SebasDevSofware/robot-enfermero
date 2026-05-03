#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "DFRobotDFPlayerMini.h"

// ============================================
// CONFIGURACIÓN DE RED (ACCESS POINT)
// ============================================
const char* ssid = "CYBERION11";
const char* password = "12345678";

// ============================================
// TIEMPOS DE LA MISIÓN (en milisegundos)
// ============================================
const unsigned long tiempoEsperaInicial = 60000;
const unsigned long tiempoAvance = 900;
const unsigned long tiempoInspeccion = 60000;
const unsigned long tiempoRetroceso = 850;

// ============================================
// PINES Y CONSTANTES
// ============================================
const int botonEmergenciaPin = 0;

#define DHTPIN 4
#define DHTTYPE DHT11  
#define RAIN_PIN 35
#define FLAME_PIN 34
#define LDR_PIN 33

#define RX_GPS 25
#define TX_GPS 26

// Pines I2C para la AMG8833
#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_AMG88xx amg;
float pixels[64];
float tempMax = 0.0;
float tempMin = 0.0;
float tempProm = 0.0;
unsigned long tiempoUltimaTermica = 0;
const unsigned long INTERVALO_TERMICO = 250;

String termicoEstado = "Inicializando...";
String termicoRecomendacion = "Esperando datos...";
String termicoAccion = "Monitoreo continuo";
int termicoUrgencia = 1;
String termicoAlerta = "";

DHT dht(DHTPIN, DHTTYPE);
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
DFRobotDFPlayerMini myDFPlayer;

enum MisionEstado {
  ESPERA_INICIAL, AVANZANDO, INSPECCION, REGRESANDO, MISION_COMPLETADA, EMERGENCIA
};
MisionEstado estadoActual = ESPERA_INICIAL;
unsigned long tiempoInicioEstado = 0;
int distanciaRadar = 0;
unsigned long ultimoEnvioDistancia = 0;

float temperatura = 0;
float humedad = 0;

// ========== VARIABLES MEJORADAS DEL SENSOR DE LLUVIA ==========
int valorLluviaCrudo = 0;
int humedadLluviaPorcentaje = 0;
int nivelLluvia = 0;
String estadoLluvia = "☀️ Completamente Seco";
String recomendacionLluvia = "🌞 Condiciones óptimas para operaciones";
String accionLluvia = "✅ Operaciones normales";
int urgenciaLluvia = 1;

// Variables de lógica difusa para lluvia
float membresiaSeco = 0, membresiaLeve = 0, membresiaModerado = 0, membresiaIntenso = 0;
int historialLluvia[30];
int historialIndexLluvia = 0;
String tendenciaLluvia = "Estable";
int valorMinLluvia = 4095;
int valorMaxLluvia = 0;
int valorPromedioLluvia = 0;

// Llama
int intensidadLlama = 0;
String nivelLlama = "Sin Llama";
String estadoLlama = "✅ Ambiente seguro";
String recomendacionLlama = "Área segura";
String accionLlama = "Continuar monitoreo";
int urgenciaLlama = 1;

// LDR
int intensidadSolar = 0;
String nivelLuz = "Normal";
String estadoLuz = "🔍 Monitoreando";
String recomendacionLuz = "Condiciones normales";
String accionLuz = "Operación estándar";
int urgenciaLuz = 1;
String iconoLuz = "🌤️";

// GPS
String latitudGMS = "Buscando...";
String longitudGMS = "Buscando...";
String analisisGeografico = "🛰️ SINCRONIZANDO...";
String iconoMundo = "🌍❓";

// MQ-2
int mq2ValorCrudo = 0;
int mq2Concentracion = 0;
int mq2Nivel = 0;
String mq2TipoGas = "Esperando datos...";
String mq2Estado = "Esperando calentamiento...";
String mq2Recomendacion = "Sensor en precalentamiento (60s)";
String mq2Accion = "Esperar calentamiento";
int mq2Urgencia = 1;
bool mq2Disponible = false;

// ========== VARIABLES DEL SENSOR DE COLOR TCS34725 ==========
String colorDetectado = "Esperando...";
String nivelAlertaColor = "Verde";
String estadoColor = "🟢 MONITOREANDO";
String recomendacionColor = "Área segura - Continuar operaciones";
String accionColor = "Monitoreo estándar";
int urgenciaColor = 1;
String iconoColor = "🟢";
int colorR = 0, colorG = 0, colorB = 0;

WebServer server(80);
#define UART Serial2

// Variables para la cámara térmica
bool camaraTermicaFuncional = false;
unsigned long ultimoReintentoCamara = 0;
const unsigned long INTERVALO_REINTENTO_CAMARA = 5000;
int intentosCamara = 0;

void enviarComando(char comando);
void manejarWeb();
void manejarEmergencia();
void verificarBotonFisico();
void actualizarMision();
String obtenerEstadoTexto();
String obtenerColorRadar();
String obtenerTextoRadar();
unsigned long getTiempoActualEstado();
int getSegundosRestantes();
void leerSensorLluvia();
void leerSensorLlama();
void leerSensorLDR();
void leerGPS();
void leerCamaraTermica();
String convertirAGMS(double valor, bool esLatitud);
String sistemaExpertoGPS(double lat, double lon);
String evaluarTemperaturaIA(float t);
String evaluarHumedadIA(float h);
String getColorTermico(float temp);
void actualizarSistemaExpertoTermico();
void actualizarSemaforoPorSensores();
void inicializarCamaraTermica();

// Prototipos de funciones de lluvia
void actualizarHistorialLluvia();
float membresiaSecoFunc(int valor);
float membresiaLeveFunc(int valor);
float membresiaModeradoFunc(int valor);
float membresiaIntensoFunc(int valor);
void sistemaExpertoLluvia();
String getEstadoLluviaTexto();
String getNivelLluviaTexto();
String getRecomendacionLluvia();
String getAccionLluvia();
int getUrgenciaLluvia();

void inicializarCamaraTermica() {
  Serial.println("🔍 Inicializando cámara térmica AMG8833...");
  
  delay(500);
  
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(200);
  
  byte error, address;
  int nDevices = 0;
  Serial.println("🔍 Escaneando bus I2C...");
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("✅ Dispositivo I2C en 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      if (address == 0x69) Serial.println(" (AMG8833)");
      else Serial.println();
      nDevices++;
    }
  }
  if (nDevices == 0) {
    Serial.println("⚠️ No hay dispositivos I2C");
  }
  
  bool sensorInicializado = false;
  
  Serial.println("🔧 Intento 1 de 3...");
  if (amg.begin()) {
    sensorInicializado = true;
    Serial.println("✅ AMG8833 inicializada en primer intento!");
  } else {
    Serial.println("⚠️ Primer intento fallido, reintentando...");
    delay(300);
    
    Wire.end();
    delay(100);
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(200);
    
    Serial.println("🔧 Intento 2 de 3...");
    if (amg.begin()) {
      sensorInicializado = true;
      Serial.println("✅ AMG8833 inicializada en segundo intento!");
    } else {
      delay(500);
      Serial.println("🔧 Intento 3 de 3...");
      if (amg.begin()) {
        sensorInicializado = true;
        Serial.println("✅ AMG8833 inicializada en tercer intento!");
      }
    }
  }
  
  if (sensorInicializado) {
    camaraTermicaFuncional = true;
    intentosCamara = 0;
    delay(200);
    amg.readPixels(pixels);
    Serial.println("✅ Cámara térmica lista!");
  } else {
    camaraTermicaFuncional = false;
    Serial.println("⚠️ AMG8833 NO detectada - Usando SIMULACIÓN");
    Serial.println("💡 Verificar:");
    Serial.println("   - Conexión VCC a 3.3V (NO 5V)");
    Serial.println("   - SDA a GPIO21, SCL a GPIO22");
    Serial.println("   - Alimentación estable");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n========================================");
  Serial.println("=== CYBERION - GUARDIANS OF THE SOIL ===");
  Serial.println("=== ESP32 MAESTRO CON IA DIFUSA (PDVSA) ===");
  Serial.println("=== CON SENSOR DE COLOR TCS34725 ===");
  Serial.println("========================================");
  Serial.println("🔄 Versión completa - Todos los sensores activos");
  Serial.println("========================================\n");

  pinMode(botonEmergenciaPin, INPUT_PULLUP);

  dht.begin();
  delay(100);
  
  pinMode(RAIN_PIN, INPUT);
  pinMode(FLAME_PIN, INPUT);
  pinMode(LDR_PIN, INPUT_PULLUP);
  
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  SerialGPS.begin(9600, SERIAL_8N1, RX_GPS, TX_GPS);
  UART.begin(115200, SERIAL_8N1, 16, 17);
  
  delay(300);
  inicializarCamaraTermica();
  
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("✅ AP Listo. IP: ");
  Serial.println(IP);

  for (int i = 0; i < 30; i++) historialLluvia[i] = 0;

  server.on("/", manejarWeb);
  server.on("/emergencia", manejarEmergencia);
  server.on("/estado", []() {
    String json = "{\"estado\":\"" + obtenerEstadoTexto() + "\",";
    json += "\"distancia\":" + String(distanciaRadar) + ",";
    json += "\"color\":\"" + obtenerColorRadar() + "\",";
    json += "\"texto\":\"" + obtenerTextoRadar() + "\",";
    json += "\"segundos\":" + String(getSegundosRestantes()) + "}";
    server.send(200, "application/json", json);
  });
  
  server.on("/datos_amb", []() {
    String json = "{";
    json += "\"temp\":" + String(temperatura) + ",";
    json += "\"hum\":" + String(humedad) + ",";
    json += "\"diagTemp\":\"" + evaluarTemperaturaIA(temperatura) + "\",";
    json += "\"diagHum\":\"" + evaluarHumedadIA(humedad) + "\",";
    json += "\"lluviaPorcentaje\":" + String(humedadLluviaPorcentaje) + ",";
    json += "\"nivelLluvia\":\"" + getNivelLluviaTexto() + "\",";
    json += "\"estadoLluvia\":\"" + getEstadoLluviaTexto() + "\",";
    json += "\"recomendacionLluvia\":\"" + getRecomendacionLluvia() + "\",";
    json += "\"accionLluvia\":\"" + getAccionLluvia() + "\",";
    json += "\"urgenciaLluvia\":" + String(getUrgenciaLluvia()) + ",";
    json += "\"tendenciaLluvia\":\"" + tendenciaLluvia + "\",";
    json += "\"membresiaSeco\":" + String(membresiaSeco, 0) + ",";
    json += "\"membresiaLeve\":" + String(membresiaLeve, 0) + ",";
    json += "\"membresiaModerado\":" + String(membresiaModerado, 0) + ",";
    json += "\"membresiaIntenso\":" + String(membresiaIntenso, 0) + ",";
    json += "\"llamaIntensidad\":" + String(intensidadLlama) + ",";
    json += "\"nivelLlama\":\"" + nivelLlama + "\",";
    json += "\"estadoLlama\":\"" + estadoLlama + "\",";
    json += "\"recomendacionLlama\":\"" + recomendacionLlama + "\",";
    json += "\"accionLlama\":\"" + accionLlama + "\",";
    json += "\"urgenciaLlama\":" + String(urgenciaLlama) + ",";
    json += "\"intensidadSolar\":" + String(intensidadSolar) + ",";
    json += "\"nivelLuz\":\"" + nivelLuz + "\",";
    json += "\"estadoLuz\":\"" + estadoLuz + "\",";
    json += "\"recomendacionLuz\":\"" + recomendacionLuz + "\",";
    json += "\"accionLuz\":\"" + accionLuz + "\",";
    json += "\"urgenciaLuz\":" + String(urgenciaLuz) + ",";
    json += "\"iconoLuz\":\"" + iconoLuz + "\",";
    json += "\"latitudGMS\":\"" + latitudGMS + "\",";
    json += "\"longitudGMS\":\"" + longitudGMS + "\",";
    json += "\"analisisGeografico\":\"" + analisisGeografico + "\",";
    json += "\"iconoMundo\":\"" + iconoMundo + "\",";
    json += "\"mq2Valor\":" + String(mq2ValorCrudo) + ",";
    json += "\"mq2Concentracion\":" + String(mq2Concentracion) + ",";
    json += "\"mq2Nivel\":" + String(mq2Nivel) + ",";
    json += "\"mq2TipoGas\":\"" + mq2TipoGas + "\",";
    json += "\"mq2Estado\":\"" + mq2Estado + "\",";
    json += "\"mq2Recomendacion\":\"" + mq2Recomendacion + "\",";
    json += "\"mq2Accion\":\"" + mq2Accion + "\",";
    json += "\"mq2Urgencia\":" + String(mq2Urgencia) + ",";
    json += "\"termico_max\":" + String(tempMax, 1) + ",";
    json += "\"termico_min\":" + String(tempMin, 1) + ",";
    json += "\"termico_prom\":" + String(tempProm, 1) + ",";
    json += "\"termico_estado\":\"" + termicoEstado + "\",";
    json += "\"termico_recomendacion\":\"" + termicoRecomendacion + "\",";
    json += "\"termico_accion\":\"" + termicoAccion + "\",";
    json += "\"termico_urgencia\":" + String(termicoUrgencia) + ",";
    json += "\"termico_alerta\":\"" + termicoAlerta + "\",";
    json += "\"color_detectado\":\"" + colorDetectado + "\",";
    json += "\"color_nivel\":\"" + nivelAlertaColor + "\",";
    json += "\"color_estado\":\"" + estadoColor + "\",";
    json += "\"color_recomendacion\":\"" + recomendacionColor + "\",";
    json += "\"color_accion\":\"" + accionColor + "\",";
    json += "\"color_urgencia\":" + String(urgenciaColor) + ",";
    json += "\"color_icono\":\"" + iconoColor + "\",";
    json += "\"color_r\":" + String(colorR) + ",";
    json += "\"color_g\":" + String(colorG) + ",";
    json += "\"color_b\":" + String(colorB) + ",";
    json += "\"pixels\":[";
    for (int i = 0; i < 64; i++) {
      json += String(pixels[i], 1);
      if (i < 63) json += ",";
    }
    json += "]";
    json += "}";
    server.send(200, "application/json", json);
  });
  
  server.begin();
  
  // === CONFIGURACIÓN MP3 (Compartiendo UART1) ===
  Serial.println("🔈 Iniciando audio de bienvenida...");
  SerialGPS.begin(9600, SERIAL_8N1, 12, 14); // Cambiamos UART1 a pines del MP3
  
  if (myDFPlayer.begin(SerialGPS)) {
    myDFPlayer.volume(25);
    myDFPlayer.playMp3Folder(2); // Reproduce mp3/0002.mp3
    
    delay(600); // Espera para que el módulo procese
    Serial.println("⏳ Esperando a que termine el audio...");
    
    // Bucle que bloquea el inicio hasta que el audio termine
    bool audioTerminado = false;
    while (!audioTerminado) {
      if (myDFPlayer.available()) {
        if (myDFPlayer.readType() == DFPlayerPlayFinished) {
          audioTerminado = true;
        }
      }
      delay(10);
    }
    Serial.println("✅ Audio finalizado.");
  } else {
    Serial.println("⚠️ No se detectó el módulo MP3.");
  }

  // Restauramos la UART1 para el GPS (pines 25 y 26)
  SerialGPS.begin(9600, SERIAL_8N1, RX_GPS, TX_GPS);
  
  tiempoInicioEstado = millis();
  Serial.println("🚀 Sistema listo. Esperando 60 segundos...");
  Serial.println("🌐 Conéctate a CYBERION11 y visita http://192.168.4.1");
}

void loop() {
  server.handleClient();
  verificarBotonFisico();
  
  if (UART.available() > 0) {
    String data = UART.readStringUntil('\n');
    data.trim();
    
    if (data.length() > 0) {
      if (data.startsWith("MQ2:")) {
        String valores = data.substring(4);
        int primerComa = valores.indexOf(',');
        int segundoComa = valores.indexOf(',', primerComa + 1);
        int tercerComa = valores.indexOf(',', segundoComa + 1);
        int cuartoComa = valores.indexOf(',', tercerComa + 1);
        int quintoComa = valores.indexOf(',', cuartoComa + 1);
        int sextoComa = valores.indexOf(',', quintoComa + 1);
        int septimoComa = valores.indexOf(',', sextoComa + 1);
        
        if (primerComa > 0 && segundoComa > 0 && tercerComa > 0) {
          mq2ValorCrudo = valores.substring(0, primerComa).toInt();
          mq2Concentracion = valores.substring(primerComa + 1, segundoComa).toInt();
          mq2Nivel = valores.substring(segundoComa + 1, tercerComa).toInt();
          mq2TipoGas = valores.substring(tercerComa + 1, cuartoComa);
          mq2Estado = valores.substring(cuartoComa + 1, quintoComa);
          mq2Recomendacion = valores.substring(quintoComa + 1, sextoComa);
          mq2Accion = valores.substring(sextoComa + 1, septimoComa);
          mq2Urgencia = valores.substring(septimoComa + 1).toInt();
          mq2Disponible = true;
        }
      } 
      else if (data.startsWith("COLOR:")) {
        String valores = data.substring(6);
        int primerComa = valores.indexOf(',');
        int segundoComa = valores.indexOf(',', primerComa + 1);
        int tercerComa = valores.indexOf(',', segundoComa + 1);
        int cuartoComa = valores.indexOf(',', tercerComa + 1);
        int quintoComa = valores.indexOf(',', cuartoComa + 1);
        int sextoComa = valores.indexOf(',', quintoComa + 1);
        int septimoComa = valores.indexOf(',', sextoComa + 1);
        int octavoComa = valores.indexOf(',', septimoComa + 1);
        int novenoComa = valores.indexOf(',', octavoComa + 1);
        
        if (primerComa > 0) {
          colorDetectado = valores.substring(0, primerComa);
          nivelAlertaColor = valores.substring(primerComa + 1, segundoComa);
          estadoColor = valores.substring(segundoComa + 1, tercerComa);
          recomendacionColor = valores.substring(tercerComa + 1, cuartoComa);
          accionColor = valores.substring(cuartoComa + 1, quintoComa);
          urgenciaColor = valores.substring(quintoComa + 1, sextoComa).toInt();
          iconoColor = valores.substring(sextoComa + 1, septimoComa);
          colorR = valores.substring(septimoComa + 1, octavoComa).toInt();
          colorG = valores.substring(octavoComa + 1, novenoComa).toInt();
          colorB = valores.substring(novenoComa + 1).toInt();
        }
      }
      else if (data != "OK" && data.length() < 5) {
        int nuevaDistancia = data.toInt();
        if (nuevaDistancia > 0 && nuevaDistancia < 400) {
          distanciaRadar = nuevaDistancia;
        }
      }
    }
  }
  
  static unsigned long ultimaLectura = 0;
  if (millis() - ultimaLectura > 2000) {
    temperatura = dht.readTemperature();
    humedad = dht.readHumidity();
    if (isnan(temperatura)) temperatura = 0;
    if (isnan(humedad)) humedad = 0;
    
    leerSensorLluvia();
    leerSensorLlama();
    leerSensorLDR();
    leerGPS();
    
    ultimaLectura = millis();
    
    static unsigned long ultimoLog = 0;
    if (millis() - ultimoLog > 3000) {
      ultimoLog = millis();
      Serial.print("🌡️ Temp: "); Serial.print(temperatura); Serial.print("°C | ");
      Serial.print("💧 Hum: "); Serial.print(humedad); Serial.print("% | ");
      Serial.print("☔ Lluvia: "); Serial.print(humedadLluviaPorcentaje); Serial.print("% | ");
      Serial.print("📊 Tendencia: "); Serial.print(tendenciaLluvia); Serial.print(" | ");
      Serial.print("🔥 Llama: "); Serial.print(intensidadLlama); Serial.print("% | ");
      Serial.print("💡 Luz: "); Serial.print(intensidadSolar); Serial.print("% | ");
      Serial.print("🎨 Color: "); Serial.print(colorDetectado); Serial.print(" | ");
      if (mq2Disponible) {
        Serial.print("💨 Gas: "); Serial.print(mq2Concentracion); Serial.print("% | ");
      }
      if (camaraTermicaFuncional) {
        Serial.print("🔥 Térmica Max: "); Serial.print(tempMax, 1); Serial.print("°C");
      } else {
        Serial.print("⚠️ Térmica: SIMULACIÓN");
      }
      Serial.println();
    }
  }
  
  if (!camaraTermicaFuncional && (millis() - ultimoReintentoCamara > INTERVALO_REINTENTO_CAMARA)) {
    ultimoReintentoCamara = millis();
    intentosCamara++;
    if (intentosCamara <= 3) {
      Serial.print("🔄 Reintento cámara térmica (");
      Serial.print(intentosCamara);
      Serial.println("/3)...");
      inicializarCamaraTermica();
    }
  }
  
  if (camaraTermicaFuncional && (millis() - tiempoUltimaTermica >= INTERVALO_TERMICO)) {
    leerCamaraTermica();
    actualizarSistemaExpertoTermico();
    tiempoUltimaTermica = millis();
  } else if (!camaraTermicaFuncional) {
    static unsigned long ultimaSimulacion = 0;
    if (millis() - ultimaSimulacion >= 5000) {
      ultimaSimulacion = millis();
      float baseTemp = temperatura > 0 ? temperatura : 25.0;
      for (int i = 0; i < 64; i++) {
        int fila = i / 8;
        int col = i % 8;
        float centroX = 3.5;
        float centroY = 3.5;
        float distancia = sqrt(pow(fila - centroY, 2) + pow(col - centroX, 2));
        float temp = baseTemp + 8.0 * exp(-distancia * 0.5);
        pixels[i] = temp;
      }
      tempMax = baseTemp + 8.0;
      tempMin = baseTemp;
      tempProm = baseTemp + 2.5;
      termicoEstado = "⚠️ SENSOR NO DETECTADO - Modo simulación";
      termicoRecomendacion = "Verificar conexiones I2C del sensor AMG8833";
      termicoAccion = "Revisar cables y alimentación (3.3V)";
      termicoAlerta = "🔌 Sensor térmico no conectado";
    }
  }
  
  actualizarSemaforoPorSensores();
  actualizarMision();
  delay(5);
}

// ============================================
// SENSOR DE LLUVIA - LÓGICA DIFUSA COMPLETA
// ============================================
void leerSensorLluvia() {
  valorLluviaCrudo = analogRead(RAIN_PIN);
  humedadLluviaPorcentaje = map(valorLluviaCrudo, 0, 4095, 0, 100);
  humedadLluviaPorcentaje = constrain(humedadLluviaPorcentaje, 0, 100);
  
  if (valorLluviaCrudo < valorMinLluvia) valorMinLluvia = valorLluviaCrudo;
  if (valorLluviaCrudo > valorMaxLluvia) valorMaxLluvia = valorLluviaCrudo;
  
  actualizarHistorialLluvia();
  sistemaExpertoLluvia();
}

void actualizarHistorialLluvia() {
  historialLluvia[historialIndexLluvia] = humedadLluviaPorcentaje;
  historialIndexLluvia = (historialIndexLluvia + 1) % 30;
  
  long suma = 0;
  for (int i = 0; i < 30; i++) {
    suma += historialLluvia[i];
  }
  valorPromedioLluvia = suma / 30;
  
  if (historialIndexLluvia >= 10) {
    int ultimos5 = 0, anteriores5 = 0;
    for (int i = 0; i < 5; i++) {
      ultimos5 += historialLluvia[(historialIndexLluvia - 1 - i + 30) % 30];
      anteriores5 += historialLluvia[(historialIndexLluvia - 6 - i + 30) % 30];
    }
    ultimos5 /= 5;
    anteriores5 /= 5;
    
    int diferencia = ultimos5 - anteriores5;
    
    if (abs(diferencia) < 3) {
      tendenciaLluvia = "Estable";
    } else if (diferencia > 0) {
      tendenciaLluvia = "Aumentando";
    } else {
      tendenciaLluvia = "Disminuyendo";
    }
  }
}

float membresiaSecoFunc(int valor) {
  if (valor <= 20) return 1.0;
  if (valor >= 40) return 0.0;
  return (40.0 - valor) / 20.0;
}

float membresiaLeveFunc(int valor) {
  if (valor <= 20 || valor >= 70) return 0.0;
  if (valor >= 40 && valor <= 50) return 1.0;
  if (valor < 40) return (valor - 20) / 20.0;
  return (70.0 - valor) / 20.0;
}

float membresiaModeradoFunc(int valor) {
  if (valor <= 50 || valor >= 90) return 0.0;
  if (valor >= 70 && valor <= 80) return 1.0;
  if (valor < 70) return (valor - 50) / 20.0;
  return (90.0 - valor) / 20.0;
}

float membresiaIntensoFunc(int valor) {
  if (valor <= 80) return 0.0;
  if (valor >= 95) return 1.0;
  return (valor - 80) / 15.0;
}

void sistemaExpertoLluvia() {
  membresiaSeco = membresiaSecoFunc(humedadLluviaPorcentaje) * 100;
  membresiaLeve = membresiaLeveFunc(humedadLluviaPorcentaje) * 100;
  membresiaModerado = membresiaModeradoFunc(humedadLluviaPorcentaje) * 100;
  membresiaIntenso = membresiaIntensoFunc(humedadLluviaPorcentaje) * 100;
  
  if (tendenciaLluvia == "Aumentando") {
    membresiaLeve *= 1.2;
    membresiaModerado *= 1.2;
    membresiaIntenso *= 1.1;
  } else if (tendenciaLluvia == "Disminuyendo") {
    membresiaSeco *= 1.2;
    membresiaLeve *= 0.9;
  }
  
  membresiaSeco = min(membresiaSeco, 100.0f);
  membresiaLeve = min(membresiaLeve, 100.0f);
  membresiaModerado = min(membresiaModerado, 100.0f);
  membresiaIntenso = min(membresiaIntenso, 100.0f);
  
  if (membresiaIntenso > 50) nivelLluvia = 3;
  else if (membresiaModerado > 50) nivelLluvia = 2;
  else if (membresiaLeve > 50) nivelLluvia = 1;
  else nivelLluvia = 0;
}

String getNivelLluviaTexto() {
  switch(nivelLluvia) {
    case 0: return "Seco";
    case 1: return "Leve";
    case 2: return "Moderado";
    case 3: return "Intenso";
    default: return "Desconocido";
  }
}

String getEstadoLluviaTexto() {
  if (membresiaIntenso > 50) return "🌊 ¡LLUVIA INTENSA! Tormenta";
  if (membresiaModerado > 50) return "🌧️ Lluvia Moderada";
  if (membresiaLeve > 50) return "⛅ Lluvia Leve";
  if (membresiaSeco > 50) return "☀️ Completamente Seco";
  if (membresiaModerado > 30) return "🌦️ Posible lluvia moderada";
  if (membresiaLeve > 30) return "☁️ Posible llovizna";
  return "🌤️ Variable";
}

String getRecomendacionLluvia() {
  if (membresiaIntenso > 50) return "🌊 ¡LLUVIA INTENSA! - QUÉDATE EN CASA, evita salir";
  if (membresiaModerado > 70) return "🌦️ Lluvia moderada en aumento - Busca refugio si es posible";
  if (membresiaModerado > 50) return "🌧️ Probabilidad de lluvia moderada - Mejor lleva paraguas";
  if (membresiaLeve > 70) return "☁️ Aumento de humedad, posible lluvia - Prepárate";
  if (membresiaLeve > 50) return "⛅ Posibilidad de llovizna leve - Lleva un paraguas";
  if (membresiaSeco > 70) return "🌞 Clima despejado, sin riesgo de lluvia";
  return "🌤️ Despejado pero aumentando humedad - Disfruta el buen tiempo";
}

String getAccionLluvia() {
  if (membresiaIntenso > 50) return "🚨 Paro de operaciones - Buscar refugio";
  if (membresiaModerado > 70) return "🌂 Operar con precaución - Buscar refugio";
  if (membresiaModerado > 50) return "🌂 Mejor llevar paraguas y impermeable";
  if (membresiaLeve > 70) return "🌂 Lleva paraguas por si acaso";
  if (membresiaLeve > 50) return "🌂 Lleva un paraguas por si acaso";
  if (membresiaSeco > 70) return "✅ Operaciones normales";
  return "✅ Disfruta el buen tiempo mientras dure";
}

int getUrgenciaLluvia() {
  if (membresiaIntenso > 50) return 5;
  if (membresiaModerado > 70) return 4;
  if (membresiaModerado > 50) return 3;
  if (membresiaLeve > 70) return 3;
  if (membresiaLeve > 50) return 2;
  if (membresiaSeco > 70) return 1;
  return 2;
}

// ============================================
// SENSOR DE LLAMA (KY-026)
// ============================================
void leerSensorLlama() {
  int valorLlamaCrudo = analogRead(FLAME_PIN);
  
  if (valorLlamaCrudo < 1200) {
    nivelLlama = "Intensa";
    estadoLlama = "🔥 ¡INCENDIO ACTIVO! - EMERGENCIA TOTAL";
    recomendacionLlama = "🚨 ACTIVAR PLAN DE CONTINGENCIA - EVACUACIÓN INMEDIATA";
    accionLlama = "LLAMAR A BOMBEROS - NO ACERCARSE";
    urgenciaLlama = 5;
  } 
  else if (valorLlamaCrudo < 2800) {
    nivelLlama = "Moderada";
    estadoLlama = "⚠️ CALOR SOSPECHOSO - PRECAUCIÓN";
    recomendacionLlama = "⚠️ Posible foco de ignición - Inspeccionar área";
    accionLlama = "Activar extintor portátil si es necesario";
    urgenciaLlama = 2;
  } 
  else {
    nivelLlama = "Sin Llama";
    estadoLlama = "✅ AMBIENTE SEGURO - NINGUNA LLAMA DETECTADA";
    recomendacionLlama = "✅ Área segura según norma COVENIN";
    accionLlama = "Continuar monitoreo";
    urgenciaLlama = 1;
  }
  
  intensidadLlama = map(valorLlamaCrudo, 4095, 0, 0, 100);
  intensidadLlama = constrain(intensidadLlama, 0, 100);
}

// ============================================
// SENSOR LDR
// ============================================
void leerSensorLDR() {
  int valorLDRDigital = digitalRead(LDR_PIN);
  
  if (valorLDRDigital == 0) {
    intensidadSolar = 90;
    nivelLuz = "Luz Adecuada";
    estadoLuz = "☀️ ILUMINACIÓN ADECUADA";
    recomendacionLuz = "✅ Condiciones óptimas para trabajo";
    accionLuz = "Operaciones normales";
    iconoLuz = "☀️☀️☀️";
    urgenciaLuz = 1;
  } 
  else {
    intensidadSolar = 10;
    nivelLuz = "Oscuridad";
    estadoLuz = "🌙 OSCURIDAD - ILUMINACIÓN INSUFICIENTE";
    recomendacionLuz = "⚠️ Normativa: Activar iluminación de emergencia";
    accionLuz = "🔦 Encender luces de seguridad";
    iconoLuz = "🌙🌙";
    urgenciaLuz = 2;
  }
}

// ============================================
// SEMÁFORO VISUAL
// ============================================
void actualizarSemaforoPorSensores() {
  int urgenciaSemaforo = 1;
  
  if (urgenciaLlama >= 4) {
    urgenciaSemaforo = 3;
  }
  else if (urgenciaLlama >= 2) {
    urgenciaSemaforo = max(urgenciaSemaforo, 2);
  }
  
  if (temperatura > 50 || humedad > 85) {
    urgenciaSemaforo = 3;
  }
  else if (temperatura > 42 || (temperatura < 15 && temperatura > 0) || (humedad < 25 && humedad > 0)) {
    urgenciaSemaforo = max(urgenciaSemaforo, 2);
  }
  
  if (tempMax > 50) {
    urgenciaSemaforo = 3;
  }
  
  if (urgenciaColor >= 4) {
    urgenciaSemaforo = 3;
  }
  else if (urgenciaColor >= 3) {
    urgenciaSemaforo = max(urgenciaSemaforo, 2);
  }
  
  char comandoSemaforo = 'G';
  if (urgenciaSemaforo == 3) comandoSemaforo = 'B';
  else if (urgenciaSemaforo == 2) comandoSemaforo = 'Y';

  static char ultimoComandoSemaforo = ' ';
  if (comandoSemaforo != ultimoComandoSemaforo) {
    UART.print(comandoSemaforo);
    ultimoComandoSemaforo = comandoSemaforo;
    
    if (comandoSemaforo == 'G') Serial.println("🟢 SEMÁFORO: VERDE");
    else if (comandoSemaforo == 'Y') Serial.println("🟡 SEMÁFORO: AMARILLO");
    else if (comandoSemaforo == 'B') Serial.println("🔴 SEMÁFORO: ROJO");
  }
}

// ============================================
// CÁMARA TÉRMICA
// ============================================
void leerCamaraTermica() {
  if (!camaraTermicaFuncional) return;
  
  amg.readPixels(pixels);
  
  tempMax = -100;
  tempMin = 100;
  float suma = 0;
  int datosValidos = 0;
  
  for (int i = 0; i < 64; i++) {
    if (pixels[i] > -40 && pixels[i] < 125) {
      if (pixels[i] > tempMax) tempMax = pixels[i];
      if (pixels[i] < tempMin) tempMin = pixels[i];
      suma += pixels[i];
      datosValidos++;
    }
  }
  
  if (datosValidos > 0) {
    tempProm = suma / datosValidos;
  }
}

void actualizarSistemaExpertoTermico() {
  if (tempMax >= 70) {
    termicoEstado = "🔥🔥 CRÍTICO: PUNTO DE IGNICIÓN";
    termicoRecomendacion = "🚨 ACTIVAR PLAN DE EMERGENCIA - EVACUACIÓN TOTAL";
    termicoAccion = "LLAMAR A BOMBEROS - EVACUAR ZONA";
    termicoUrgencia = 5;
    termicoAlerta = "🚨🚨🚨 ALERTA MÁXIMA - PELIGRO DE EXPLOSIÓN 🚨🚨🚨";
  }
  else if (tempMax >= 60) {
    termicoEstado = "🔥 ALERTA MÁXIMA: PUNTO CALIENTE PELIGROSO";
    termicoRecomendacion = "🚨 RIESGO DE INCENDIO - Activar protocolo PDVSA";
    termicoAccion = "Evacuar área - Equipo de extinción listo";
    termicoUrgencia = 5;
    termicoAlerta = "🔥🔥 ALERTA ROJA - RIESGO EXTREMO 🔥🔥";
  }
  else if (tempMax >= 50) {
    termicoEstado = "🔴 ALERTA CRÍTICA: SOBRECALENTAMIENTO";
    termicoRecomendacion = "⚠️ Inspección inmediata del punto caliente";
    termicoAccion = "Ventilar área - Inspeccionar con cámara";
    termicoUrgencia = 4;
    termicoAlerta = "🔴🔴 RIESGO ALTO DE INCENDIO 🔴🔴";
  }
  else if (tempMax >= 45) {
    termicoEstado = "🟠 ALERTA: TEMPERATURA ELEVADA";
    termicoRecomendacion = "⚠️ Monitoreo continuo requerido";
    termicoAccion = "Inspeccionar punto caliente";
    termicoUrgencia = 3;
    termicoAlerta = "🟠 PRECAUCIÓN - SOBRECALENTAMIENTO 🟠";
  }
  else if (tempMax >= 40) {
    termicoEstado = "🟡 PRECAUCIÓN: ÁREA CÁLIDA DETECTADA";
    termicoRecomendacion = "ℹ️ Verificar condiciones operativas";
    termicoAccion = "Monitorear evolución térmica";
    termicoUrgencia = 2;
    termicoAlerta = "🟡 MONITOREAR - TEMPERATURA MODERADA 🟡";
  }
  else if (tempMax >= 35) {
    termicoEstado = "🟢 NORMAL: Temperatura dentro de rangos";
    termicoRecomendacion = "✅ Condiciones seguras";
    termicoAccion = "Continuar monitoreo rutinario";
    termicoUrgencia = 1;
    termicoAlerta = "";
  }
  else {
    termicoEstado = "🔵 TEMPERATURA BAJA: Área fría";
    termicoRecomendacion = "ℹ️ Verificar posibles fugas criogénicas";
    termicoAccion = "Inspeccionar por condensación";
    termicoUrgencia = 1;
    termicoAlerta = "";
  }
}

// ============================================
// GPS
// ============================================
void leerGPS() {
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {
        latitudGMS = convertirAGMS(gps.location.lat(), true);
        longitudGMS = convertirAGMS(gps.location.lng(), false);
        analisisGeografico = sistemaExpertoGPS(gps.location.lat(), gps.location.lng());
        iconoMundo = "🌍🛰️";
      }
    }
  }
}

String convertirAGMS(double valor, bool esLatitud) {
  if (valor == 0) return "Buscando...";
  double absValor = abs(valor);
  int grados = (int)absValor;
  double minFloat = (absValor - grados) * 60;
  int minutos = (int)minFloat;
  double segundos = (minFloat - minutos) * 60;
  String direccion = "";
  if (esLatitud) direccion = (valor >= 0) ? "N" : "S";
  else direccion = (valor >= 0) ? "E" : "O";
  return String(grados) + "° " + String(minutos) + "' " + String(segundos, 1) + "'' " + direccion;
}

String sistemaExpertoGPS(double lat, double lon) {
  if (lat == 0) return "🛰️ BUSCANDO SEÑAL GPS";
  return "📍 ZONA OPERATIVA PDVSA - Monitoreo activo";
}

// ============================================
// DHT11
// ============================================
String evaluarTemperaturaIA(float t) {
  if (t > 50) return "⚠️ ALERTA CRÍTICA: Temperatura EXTREMA";
  if (t > 45) return "⚠️ ALERTA: Sobrecarga térmica";
  if (t > 42) return "🔴 ALERTA: Temperatura elevada - Hidratación obligatoria";
  if (t < 15 && t > 0) return "ℹ️ AVISO: Temperatura baja - Usar EPP térmico";
  if (t <= 0) return "⚠️ ERROR: Sensor no detectado - Verificar conexión";
  return "✅ NORMAL: Temperatura dentro de rangos operativos";
}

String evaluarHumedadIA(float h) {
  if (h < 25 && h > 0) return "⚡ RIESGO: Ambiente seco - Riesgo de descargas electrostáticas";
  if (h > 85) return "💧 ALERTA: Humedad extrema - Riesgo de corrosión";
  if (h <= 0) return "⚠️ ERROR: Sensor no detectado - Verificar conexión";
  return "✅ SEGURO: Humedad dentro de rangos operativos";
}

String getColorTermico(float temp) {
  if (temp >= 50) return "#FF0000";
  if (temp >= 45) return "#FF6600";
  if (temp >= 40) return "#FFFF00";
  if (temp >= 35) return "#00FF00";
  if (temp >= 30) return "#00AAFF";
  if (temp >= 25) return "#0066FF";
  return "#0000FF";
}

// ============================================
// FUNCIONES AUXILIARES
// ============================================
unsigned long getTiempoActualEstado() {
  if (estadoActual == EMERGENCIA || estadoActual == MISION_COMPLETADA) return 0;
  return millis() - tiempoInicioEstado;
}

int getSegundosRestantes() {
  unsigned long transcurrido = getTiempoActualEstado();
  unsigned long objetivo = 0;
  switch (estadoActual) {
    case ESPERA_INICIAL: objetivo = tiempoEsperaInicial; break;
    case INSPECCION: objetivo = tiempoInspeccion; break;
    default: return 0;
  }
  if (objetivo > 0 && transcurrido < objetivo) return (objetivo - transcurrido) / 1000;
  return 0;
}

String obtenerColorRadar() {
  if (distanciaRadar >= 10 && distanciaRadar <= 20) return "#ff0000";
  else if (distanciaRadar > 0 && distanciaRadar <= 30) return "#ff8c00";
  else if (distanciaRadar <= 40) return "#ffff00";
  else if (distanciaRadar <= 50) return "#00ff00";
  else if (distanciaRadar <= 60) return "#0000ff";
  else if (distanciaRadar <= 70) return "#ffffff";
  else return "#555555";
}

String obtenerTextoRadar() {
  if (distanciaRadar >= 10 && distanciaRadar <= 20) return "🔴 OBSTÁCULO MUY CERCANO";
  else if (distanciaRadar > 0 && distanciaRadar <= 30) return "🟠 OBSTÁCULO PRÓXIMO";
  else if (distanciaRadar <= 40) return "🟡 OBJETO DETECTADO";
  else if (distanciaRadar <= 50) return "🟢 OBJETO A DISTANCIA SEGURA";
  else if (distanciaRadar <= 60) return "🔵 OBJETO LEJANO";
  else if (distanciaRadar <= 70) return "⚪ OBJETO MUY LEJANO";
  else return "📡 FUERA DE RANGO";
}

String obtenerEstadoTexto() {
  switch (estadoActual) {
    case ESPERA_INICIAL: return "⏳ EN ESPERA INICIAL";
    case AVANZANDO: return "🚀 AVANZANDO";
    case INSPECCION: return "🔬 INSPECCIÓN ACTIVA";
    case REGRESANDO: return "⬅️ REGRESANDO";
    case MISION_COMPLETADA: return "🏁 MISIÓN COMPLETADA";
    case EMERGENCIA: return "🚨 PARO DE EMERGENCIA 🚨";
    default: return "🟢 SISTEMA LISTO";
  }
}

void enviarComando(char comando) {
  UART.print(comando);
  Serial.print("📤 Comando: ");
  Serial.println(comando);
}

void manejarEmergencia() {
  if (estadoActual != EMERGENCIA) {
    Serial.println("!!! 🚨 EMERGENCIA VÍA WEB 🚨 !!!");
    estadoActual = EMERGENCIA;
    enviarComando('E');
  }
  server.send(200, "text/plain", "EMERGENCIA ACTIVADA");
}

void verificarBotonFisico() {
  if (digitalRead(botonEmergenciaPin) == LOW) {
    delay(50);
    if (digitalRead(botonEmergenciaPin) == LOW) {
      if (estadoActual != EMERGENCIA) {
        Serial.println("!!! 🚨 BOTÓN FÍSICO DE EMERGENCIA 🚨 !!!");
        estadoActual = EMERGENCIA;
        enviarComando('E');
      }
      while (digitalRead(botonEmergenciaPin) == LOW) delay(50);
    }
  }
}

void actualizarMision() {
  if (estadoActual == EMERGENCIA || estadoActual == MISION_COMPLETADA) return;
  
  unsigned long ahora = millis();
  unsigned long transcurrido = ahora - tiempoInicioEstado;

  switch (estadoActual) {
    case ESPERA_INICIAL:
      if (transcurrido >= tiempoEsperaInicial) {
        enviarComando('A');
        estadoActual = AVANZANDO;
        tiempoInicioEstado = ahora;
        Serial.println(">>> 🚀 INICIANDO AVANCE");
      }
      break;
    case AVANZANDO:
      if (transcurrido >= tiempoAvance) {
        enviarComando('I');
        estadoActual = INSPECCION;
        tiempoInicioEstado = ahora;
        Serial.println(">>> 🔬 INICIANDO INSPECCIÓN");
      }
      break;
    case INSPECCION:
      if (transcurrido >= tiempoInspeccion) {
        enviarComando('R');
        estadoActual = REGRESANDO;
        tiempoInicioEstado = ahora;
        Serial.println(">>> ⬅️ INICIANDO REGRESO");
      }
      break;
    case REGRESANDO:
      if (transcurrido >= tiempoRetroceso) {
        enviarComando('D');
        estadoActual = MISION_COMPLETADA;
        Serial.println(">>> 🏁 MISIÓN COMPLETADA");

        // Sonar audio de fin de misión
        SerialGPS.begin(9600, SERIAL_8N1, 12, 14);
        myDFPlayer.playMp3Folder(1); // Reproduce mp3/0001.mp3
        delay(200); // Breve espera para asegurar que el comando se envíe
        SerialGPS.begin(9600, SERIAL_8N1, RX_GPS, TX_GPS); // Restaurar GPS
      }
      break;
    default: break;
  }
}

// ============================================
// INTERFAZ WEB
// ============================================
void manejarWeb() {
  unsigned long transcurrido = getTiempoActualEstado();
  unsigned long objetivo = 0;
  bool mostrarContador = false;
  
  switch (estadoActual) {
    case ESPERA_INICIAL: objetivo = tiempoEsperaInicial; mostrarContador = true; break;
    case INSPECCION: objetivo = tiempoInspeccion; mostrarContador = true; break;
    default: break;
  }
  
  int segundosRestantes = 0;
  if (objetivo > 0 && transcurrido < objetivo) segundosRestantes = (objetivo - transcurrido) / 1000;
  
  String colorRadar = obtenerColorRadar();
  String textoRadar = obtenerTextoRadar();
  
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width, initial-scale=1, user-scalable=no'>
<title>CYBERION - Sistema de Inspección Industrial</title>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body { background: linear-gradient(135deg, #000000 0%, #0a0505 100%); min-height: 100vh; display: flex; justify-content: center; align-items: center; font-family: 'Courier New', monospace; padding: 20px; }
.container { max-width: 1800px; width: 100%; margin: auto; }
.main-card { background: rgba(5, 5, 5, 0.95); border-radius: 30px; padding: 25px; box-shadow: 0 0 50px rgba(255, 215, 0, 0.3); border: 1px solid #ffd700; }
.titulo { text-align: center; margin-bottom: 20px; }
.titulo h1 { font-size: 2.8em; background: linear-gradient(135deg, #ffd700 0%, #0026ff 50%, #ff0000 100%); -webkit-background-clip: text; -webkit-text-fill-color: transparent; background-clip: text; letter-spacing: 4px; }
.titulo h3 { color: #ffd700; margin-top: 5px; }
.equipo { text-align: center; background: linear-gradient(135deg, #ffd70033, #0026ff33, #ff000033); padding: 10px; border-radius: 15px; border: 1px solid #ffd700; margin: 10px 0; }
.equipo p { margin: 5px 0; color: #ffd700; }
.badges { text-align: center; margin: 10px 0; }
.badge { display: inline-block; background: linear-gradient(135deg, #ffd70033, #0026ff33, #ff000033); padding: 4px 10px; border-radius: 15px; font-size: 0.75em; margin: 3px; color: white; font-weight: bold; border: 1px solid #ffd700; }
.tiempos-box { background: linear-gradient(135deg, #ffd70022, #0026ff22, #ff000022); border-radius: 15px; padding: 15px; margin: 15px 0; text-align: center; border-left: 4px solid #ffd700; }
.estado-texto { font-size: 1.3em; font-weight: bold; color: #ffd700; margin-bottom: 10px; }
.contador-box { background: #000000aa; border-radius: 20px; padding: 15px; margin: 10px 0; text-align: center; border: 2px solid #ffd700; }
.contador-numero { font-size: 3em; font-weight: bold; background: linear-gradient(135deg, #ffd700, #0026ff, #ff0000); -webkit-background-clip: text; -webkit-text-fill-color: transparent; background-clip: text; }
.sensors-grid { display: flex; justify-content: center; flex-wrap: wrap; gap: 20px; margin: 25px 0; }
.sensor-card { background: #111; border: 2px solid #ffd700; border-radius: 20px; padding: 20px; width: 340px; text-align: center; }
.sensor-card h3 { color: #ffd700; margin-bottom: 15px; font-size: 1.1em; }
.termometro-container { width: 40px; height: 150px; background: #222; margin: 15px auto; border-radius: 20px; position: relative; border: 2px solid #555; overflow: hidden; }
.termometro-fill { width: 100%; position: absolute; bottom: 0; background: linear-gradient(to top, #ff0000, #ff8c00); transition: height 0.3s ease; }
.gota-container { width: 60px; height: 80px; margin: 15px auto; position: relative; }
.gota-bg { width: 100%; height: 100%; background: #222; clip-path: polygon(50% 0%, 100% 70%, 80% 100%, 20% 100%, 0% 70%); position: relative; overflow: hidden; }
.gota-fill { width: 100%; position: absolute; bottom: 0; background: linear-gradient(to top, #007bff, #00c6ff); transition: height 0.3s ease; }
.ldr-container { width: 60px; height: 100px; margin: 15px auto; position: relative; }
.ldr-bg { width: 100%; height: 100%; background: #222; border-radius: 30px; position: relative; overflow: hidden; }
.ldr-fill { width: 100%; position: absolute; bottom: 0; background: linear-gradient(to top, #ffd700, #ffaa00, #ff8c00); transition: height 0.3s ease; }
.llama-container { width: 60px; height: 80px; margin: 15px auto; position: relative; }
.llama-bg { width: 100%; height: 100%; background: #222; border-radius: 30px 30px 15px 15px; position: relative; overflow: hidden; }
.llama-fill { width: 100%; position: absolute; bottom: 0; background: linear-gradient(to top, #ff4500, #ff6347, #ff8c00); transition: height 0.3s ease; }
.mq2-container { width: 60px; height: 100px; margin: 15px auto; position: relative; }
.mq2-bg { width: 100%; height: 100%; background: #222; border-radius: 20px; position: relative; overflow: hidden; }
.mq2-fill { width: 100%; position: absolute; bottom: 0; background: linear-gradient(to top, #ff8c00, #ff4500, #8b0000); transition: height 0.3s ease; }
.color-container { width: 80px; height: 80px; margin: 15px auto; border-radius: 50%; border: 3px solid #ffd700; transition: background-color 0.3s ease; background: #333; }
.color-rgb { font-size: 0.8em; color: #aaa; margin-top: 5px; }
.llama-icono, .ldr-icono, .gps-icono, .mq2-icono, .color-icono { font-size: 60px; margin: 15px 0; }
@keyframes flicker { 0% { opacity: 0.8; } 100% { opacity: 1; } }
@keyframes rotate { from { transform: rotate(0deg); } to { transform: rotate(360deg); } }
.llama-icono { animation: flicker 0.5s infinite; }
.gps-icono { animation: rotate 4s linear infinite; display: inline-block; }
.ia-diagnostico { background: #000000aa; border-left: 4px solid #ffd700; padding: 10px; margin-top: 15px; border-radius: 8px; font-size: 0.85em; color: #ffd700; text-align: left; }
.valor-numero { font-size: 2em; font-weight: bold; color: #ffd700; margin: 10px 0; }
.radar-box { border: 4px solid )rawliteral" + colorRadar + R"rawliteral(; padding: 25px; margin: 20px 0; border-radius: 20px; text-align: center; background: rgba(0, 0, 0, 0.8); transition: all 0.3s ease; box-shadow: 0 0 20px )rawliteral" + colorRadar + R"rawliteral(; }
.radar-text { font-size: 1.8em; font-weight: bold; color: )rawliteral" + colorRadar + R"rawliteral(; margin-bottom: 10px; }
.radar-distancia { font-size: 2.5em; font-weight: bold; color: )rawliteral" + colorRadar + R"rawliteral(; }
.recomendacion-box { background: linear-gradient(135deg, #1e3c72, #2a5298); padding: 20px; border-radius: 20px; margin: 15px 0; text-align: center; }
.recomendacion-titulo { color: #ffd700; font-size: 1em; margin-bottom: 8px; }
.recomendacion-texto { font-size: 1.1em; font-weight: bold; margin-bottom: 8px; }
.accion-texto { font-size: 0.95em; background: rgba(255,255,255,0.2); padding: 8px; border-radius: 20px; display: inline-block; }
.urgencia-box { margin-top: 10px; font-size: 0.9em; color: #ffd700; }
.btn-emergencia { width: 100%; background: linear-gradient(135deg, #8b0000, #ff0000); color: white; padding: 15px; font-size: 1.3em; font-weight: bold; border: none; border-radius: 40px; cursor: pointer; margin-top: 15px; transition: all 0.3s; box-shadow: 0 0 15px rgba(255, 0, 0, 0.5); }
.btn-emergencia:hover { transform: scale(1.02); box-shadow: 0 0 25px rgba(255, 0, 0, 0.8); }
.footer { text-align: center; margin-top: 15px; font-size: 0.7em; color: #ffd700aa; }
.alerta-incendio { background: linear-gradient(135deg, #8b0000, #ff0000); animation: pulse-bg 0.8s infinite; }
.alerta-gas { background: linear-gradient(135deg, #ff8c00, #ff4500); animation: pulse-bg 0.8s infinite; }
.alerta-termica { background: linear-gradient(135deg, #ff5500, #ff0000); animation: pulse-bg 0.5s infinite; }
.alerta-color-rojo { background: linear-gradient(135deg, #8b0000, #ff0000); animation: pulse-bg 0.5s infinite; }
.alerta-color-naranja { background: linear-gradient(135deg, #ff8c00, #ff5500); animation: pulse-bg 0.8s infinite; }
.alerta-color-amarillo { background: linear-gradient(135deg, #ffaa00, #ff8800); animation: pulse-bg 1s infinite; }
@keyframes pulse-bg { 0% { opacity: 0.7; } 100% { opacity: 1; } }
.gps-data { font-family: monospace; font-size: 0.9em; color: #2ecc71; text-align: left; margin: 10px 0; }
.gps-data p { margin: 5px 0; }
.termico-panel { background: #111; border: 3px solid #ff5500; border-radius: 20px; padding: 20px; margin: 20px 0; text-align: center; transition: all 0.3s ease; }
.termico-header { color: #ff5500; font-size: 1.4em; margin-bottom: 15px; }
.termico-grid { display: grid; grid-template-columns: repeat(8, 1fr); gap: 2px; max-width: 300px; margin: 15px auto; background: #000; padding: 5px; border-radius: 8px; }
.termico-pixel { aspect-ratio: 1; border-radius: 3px; border: 1px solid #333; transition: background-color 0.1s ease; }
.stats-termico { display: flex; justify-content: space-around; margin: 10px 0; font-size: 0.9em; }
.stat-box { background: #1a331a; padding: 5px; border-radius: 8px; text-align: center; flex: 1; margin: 0 3px; }
.max { color: #ff5555; }
.min { color: #5555ff; }
.prom { color: #ffff55; }
.leyenda { display: flex; justify-content: center; gap: 8px; flex-wrap: wrap; margin: 10px 0; font-size: 0.7em; }
.color-box { width: 12px; height: 12px; display: inline-block; margin-right: 2px; border-radius: 2px; }
.termico-recomendacion-box { background: linear-gradient(135deg, #2a2a2a, #1a1a1a); padding: 15px; border-radius: 15px; margin: 10px 0; text-align: center; border-left: 4px solid #ff5500; }
.termico-alerta { color: #ff5555; font-weight: bold; font-size: 0.9em; margin-top: 8px; animation: pulse-text 1s infinite; }
@keyframes pulse-text { 0% { opacity: 0.6; } 100% { opacity: 1; } }
@media (max-width: 700px) { .sensor-card { width: 100%; } .termico-grid { max-width: 280px; } }
</style>
<script>
function getColorTermico(temp) {
    if(temp >= 50) return '#FF0000';
    if(temp >= 45) return '#FF6600';
    if(temp >= 40) return '#FFFF00';
    if(temp >= 35) return '#00FF00';
    if(temp >= 30) return '#00AAFF';
    if(temp >= 25) return '#0066FF';
    return '#0000FF';
}

function dibujarTermica(pixels) {
    for(let i = 0; i < 64; i++) {
        let pixel = document.getElementById('pixel-' + i);
        if(pixel) pixel.style.backgroundColor = getColorTermico(pixels[i]);
    }
}

function actualizarRadar() {
  fetch('/estado')
    .then(r => r.json())
    .then(data => {
      document.getElementById('estadoDinamico').innerText = data.estado;
      document.getElementById('radarTexto').innerHTML = data.texto;
      document.getElementById('radarDistancia').innerHTML = data.distancia;
      const radarBox = document.getElementById('radarBox');
      radarBox.style.borderColor = data.color;
      radarBox.style.boxShadow = '0 0 20px ' + data.color;
      document.getElementById('radarTexto').style.color = data.color;
      document.getElementById('radarDistancia').style.color = data.color;
      if(data.segundos > 0) {
        document.getElementById('contadorNumero').innerHTML = data.segundos;
        document.getElementById('contadorBox').style.display = 'block';
      } else {
        document.getElementById('contadorBox').style.display = 'none';
      }
    });
}

function actualizarSensores() {
  fetch('/datos_amb')
    .then(r => r.json())
    .then(data => {
      let fillTemp = Math.min(100, Math.max(0, (data.temp / 50) * 100));
      document.getElementById('termometroFill').style.height = fillTemp + '%';
      document.getElementById('valTemp').innerHTML = data.temp.toFixed(1) + '°C';
      document.getElementById('iaTemp').innerHTML = data.diagTemp;
      
      let fillHum = Math.min(100, Math.max(0, data.hum));
      document.getElementById('gotaFill').style.height = fillHum + '%';
      document.getElementById('valHum').innerHTML = data.hum.toFixed(1) + '%';
      document.getElementById('iaHum').innerHTML = data.diagHum;
      
      let fillLuz = Math.min(100, Math.max(0, data.intensidadSolar));
      document.getElementById('ldrFill').style.height = fillLuz + '%';
      document.getElementById('valorLuz').innerHTML = data.intensidadSolar + '%';
      document.getElementById('estadoLuz').innerHTML = data.estadoLuz;
      document.getElementById('nivelLuzTexto').innerHTML = data.nivelLuz;
      document.getElementById('iconoLuz').innerHTML = data.iconoLuz;
      document.getElementById('recomendacionLuz').innerHTML = data.recomendacionLuz;
      document.getElementById('accionLuz').innerHTML = data.accionLuz;
      let puntosLuz = '●'.repeat(data.urgenciaLuz) + '○'.repeat(5-data.urgenciaLuz);
      document.getElementById('urgenciaLuz').innerHTML = 'Urgencia: ' + puntosLuz;
      
      let fillLluvia = Math.min(100, data.lluviaPorcentaje);
      document.getElementById('lluviaFill').style.height = fillLluvia + '%';
      document.getElementById('valorLluvia').innerHTML = data.lluviaPorcentaje + '%';
      document.getElementById('estadoLluvia').innerHTML = data.estadoLluvia;
      document.getElementById('nivelLluviaTexto').innerHTML = data.nivelLluvia;
      document.getElementById('recomendacionLluvia').innerHTML = data.recomendacionLluvia;
      document.getElementById('accionLluvia').innerHTML = data.accionLluvia;
      document.getElementById('tendenciaLluvia').innerHTML = data.tendenciaLluvia;
      let puntosLluvia = '●'.repeat(data.urgenciaLluvia) + '○'.repeat(5-data.urgenciaLluvia);
      document.getElementById('urgenciaLluvia').innerHTML = 'Urgencia: ' + puntosLluvia;
      
      document.getElementById('membresiaSeco').innerHTML = data.membresiaSeco + '%';
      document.getElementById('membresiaLeve').innerHTML = data.membresiaLeve + '%';
      document.getElementById('membresiaModerado').innerHTML = data.membresiaModerado + '%';
      document.getElementById('membresiaIntenso').innerHTML = data.membresiaIntenso + '%';
      document.getElementById('barSeco').style.width = data.membresiaSeco + '%';
      document.getElementById('barLeve').style.width = data.membresiaLeve + '%';
      document.getElementById('barModerado').style.width = data.membresiaModerado + '%';
      document.getElementById('barIntenso').style.width = data.membresiaIntenso + '%';
      
      let fillLlama = Math.min(100, data.llamaIntensidad);
      document.getElementById('llamaFill').style.height = fillLlama + '%';
      document.getElementById('valorLlama').innerHTML = data.llamaIntensidad + '%';
      document.getElementById('estadoLlama').innerHTML = data.estadoLlama;
      document.getElementById('nivelLlamaTexto').innerHTML = data.nivelLlama;
      document.getElementById('recomendacionLlama').innerHTML = data.recomendacionLlama;
      document.getElementById('accionLlama').innerHTML = data.accionLlama;
      let puntosLlama = '●'.repeat(data.urgenciaLlama) + '○'.repeat(5-data.urgenciaLlama);
      document.getElementById('urgenciaLlama').innerHTML = 'Urgencia: ' + puntosLlama;
      
      let llamaIcono = document.getElementById('llamaIcono');
      if(data.nivelLlama === 'Intensa') {
        llamaIcono.innerHTML = '🔥🔥🔥';
        document.getElementById('llamaCard').classList.add('alerta-incendio');
      } else if(data.nivelLlama === 'Moderada') {
        llamaIcono.innerHTML = '🔥🔥';
        document.getElementById('llamaCard').classList.remove('alerta-incendio');
      } else {
        llamaIcono.innerHTML = '💨';
        document.getElementById('llamaCard').classList.remove('alerta-incendio');
      }
      
      let fillMQ2 = Math.min(100, data.mq2Concentracion);
      document.getElementById('mq2Fill').style.height = fillMQ2 + '%';
      document.getElementById('valorMQ2').innerHTML = data.mq2Concentracion + '%';
      document.getElementById('estadoMQ2').innerHTML = data.mq2Estado;
      document.getElementById('nivelMQ2Texto').innerHTML = data.mq2TipoGas;
      document.getElementById('recomendacionMQ2').innerHTML = data.mq2Recomendacion;
      document.getElementById('accionMQ2').innerHTML = data.mq2Accion;
      let puntosMQ2 = '●'.repeat(data.mq2Urgencia) + '○'.repeat(5-data.mq2Urgencia);
      document.getElementById('urgenciaMQ2').innerHTML = 'Urgencia: ' + puntosMQ2;
      
      let mq2Icono = document.getElementById('mq2Icono');
      if(data.mq2Urgencia >= 4) {
        mq2Icono.innerHTML = '💨💨💨';
        document.getElementById('mq2Card').classList.add('alerta-gas');
      } else if(data.mq2Urgencia >= 3) {
        mq2Icono.innerHTML = '💨💨';
        document.getElementById('mq2Card').classList.remove('alerta-gas');
      } else if(data.mq2Concentracion > 20) {
        mq2Icono.innerHTML = '💨';
        document.getElementById('mq2Card').classList.remove('alerta-gas');
      } else {
        mq2Icono.innerHTML = '🌫️';
        document.getElementById('mq2Card').classList.remove('alerta-gas');
      }
      
      document.getElementById('latGMS').innerHTML = data.latitudGMS;
      document.getElementById('lonGMS').innerHTML = data.longitudGMS;
      document.getElementById('analisisGeografico').innerHTML = data.analisisGeografico;
      document.getElementById('iconoMundo').innerHTML = data.iconoMundo;
      
      document.getElementById('tmax').innerText = data.termico_max.toFixed(1) + '°C';
      document.getElementById('tmin').innerText = data.termico_min.toFixed(1) + '°C';
      document.getElementById('tprom').innerText = data.termico_prom.toFixed(1) + '°C';
      document.getElementById('termicoEstado').innerHTML = '📊 ' + data.termico_estado;
      document.getElementById('termicoRecomendacion').innerHTML = data.termico_recomendacion;
      document.getElementById('termicoAccion').innerHTML = data.termico_accion;
      let puntosTermico = '●'.repeat(data.termico_urgencia) + '○'.repeat(5-data.termico_urgencia);
      document.getElementById('termicoUrgencia').innerHTML = 'Urgencia: ' + puntosTermico;
      
      if(data.termico_alerta && data.termico_alerta !== '') {
        document.getElementById('termicoAlerta').innerHTML = data.termico_alerta;
        document.getElementById('termicoAlerta').style.display = 'block';
        document.getElementById('termicoPanel').classList.add('alerta-termica');
      } else {
        document.getElementById('termicoAlerta').innerHTML = '';
        document.getElementById('termicoAlerta').style.display = 'none';
        document.getElementById('termicoPanel').classList.remove('alerta-termica');
      }
      
      if(data.pixels) dibujarTermica(data.pixels);
      
      document.getElementById('colorDetectado').innerHTML = data.color_detectado;
      document.getElementById('colorEstado').innerHTML = data.color_estado;
      document.getElementById('colorIcono').innerHTML = data.color_icono;
      document.getElementById('colorRGB').innerHTML = 'R:' + data.color_r + ' G:' + data.color_g + ' B:' + data.color_b;
      
      let colorCircle = document.getElementById('colorCircle');
      let rgbColor = 'rgb(' + data.color_r + ',' + data.color_g + ',' + data.color_b + ')';
      colorCircle.style.backgroundColor = rgbColor;
      
      let colorCard = document.getElementById('colorCard');
      colorCard.classList.remove('alerta-color-rojo', 'alerta-color-naranja', 'alerta-color-amarillo');
      if(data.color_detectado === 'ROJO') {
        colorCard.classList.add('alerta-color-rojo');
      } else if(data.color_detectado === 'NARANJA') {
        colorCard.classList.add('alerta-color-naranja');
      } else if(data.color_detectado === 'AMARILLO') {
        colorCard.classList.add('alerta-color-amarillo');
      }
    })
    .catch(err => console.log('Error:', err));
}

window.onload = function() {
  actualizarRadar();
  actualizarSensores();
  setInterval(actualizarRadar, 500);
  setInterval(actualizarSensores, 1000);
};
</script>
</head>
<body>
<div class='container'>
<div class='main-card'>
<div class='titulo'>
<h1>CYBERION</h1>
<h3>GUARDIANS OF THE SOIL</h3>
</div>
<div class='equipo'>
<p> Victoria Silva | Sharlotte Rivero | Analia Sequea</p>
</div>
<div class='badges'>
<span class='badge'>🏆 ORC 2026</span>
<span class='badge'>⚙️ MOTOR HIDROCARBUROS</span>
<span class='badge'>🌊 IA DIFUSA </span>
<span class='badge'>🔥 KY-026 FLAME</span>
<span class='badge'>💡 KY-018 LDR</span>
<span class='badge'>🛰️ NEO-6M GPS</span>
<span class='badge'>💨 MQ-2 GAS</span>
<span class='badge'>🔥 AMG8833 TÉRMICA</span>
<span class='badge'>🎨 TCS34725 COLOR</span>
</div>

<div class='tiempos-box'>
<div class='estado-texto' id='estadoDinamico'>)rawliteral" + obtenerEstadoTexto() + R"rawliteral(</div>
<div id='contadorBox' class='contador-box' style='display: )rawliteral" + String(mostrarContador ? "block" : "none") + R"rawliteral('>
<div class='contador-numero' id='contadorNumero'>)rawliteral" + String(segundosRestantes) + R"rawliteral(</div>
<div class='contador-label'>SEGUNDOS RESTANTES</div>
</div>
</div>

<div class='radar-box' id='radarBox'>
<div class='radar-text' id='radarTexto'>)rawliteral" + textoRadar + R"rawliteral(</div>
<div><span class='radar-distancia' id='radarDistancia'>)rawliteral" + String(distanciaRadar) + R"rawliteral(</span><span> cm</span></div>
</div>

<div class='sensors-grid'>
<div class='sensor-card'>
<h3>🌡️ TEMPERATURA INDUSTRIAL</h3>
<div class='termometro-container'><div id='termometroFill' class='termometro-fill' style='height:0%'></div></div>
<div class='valor-numero' id='valTemp'>--°C</div>
<div class='ia-diagnostico' id='iaTemp'>Esperando...</div>
</div>

<div class='sensor-card'>
<h3>💧 HUMEDAD AMBIENTAL</h3>
<div class='gota-container'><div class='gota-bg'><div id='gotaFill' class='gota-fill' style='height:0%'></div></div></div>
<div class='valor-numero' id='valHum'>--%</div>
<div class='ia-diagnostico' id='iaHum'>Esperando...</div>
</div>

<div class='sensor-card'>
<h3>🛰️ GPS NEO-6M</h3>
<div id='iconoMundo' class='gps-icono'>🌍</div>
<div class='gps-data'>
<p><b>LATITUD:</b> <span id='latGMS'>Buscando...</span></p>
<p><b>LONGITUD:</b> <span id='lonGMS'>Buscando...</span></p>
</div>
<div class='ia-diagnostico' id='analisisGeografico'>Esperando señal...</div>
</div>

<div class='sensor-card'>
<h3>💡 SENSOR LDR KY-018</h3>
<div id='iconoLuz' class='ldr-icono'>🌤️</div>
<div class='ldr-container'><div class='ldr-bg'><div id='ldrFill' class='ldr-fill' style='height:0%'></div></div></div>
<div class='valor-numero' id='valorLuz'>--%</div>
<div id='estadoLuz' style='color:#ffd700; font-size:0.9em;'>---</div>
<div id='nivelLuzTexto' style='color:#aaa;'>---</div>
<div class='recomendacion-box'>
<div class='recomendacion-titulo'>🎯 RECOMENDACIÓN</div>
<div class='recomendacion-texto' id='recomendacionLuz'>---</div>
<div class='accion-texto' id='accionLuz'>---</div>
<div class='urgencia-box' id='urgenciaLuz'>Urgencia: ○○○○○</div>
</div>
</div>

<div class='sensor-card' id='llamaCard'>
<h3>🔥 SENSOR DE LLAMA KY-026</h3>
<div id='llamaIcono' class='llama-icono'>💨</div>
<div class='llama-container'><div class='llama-bg'><div id='llamaFill' class='llama-fill' style='height:0%'></div></div></div>
<div class='valor-numero' id='valorLlama'>--%</div>
<div id='estadoLlama' style='color:#ffd700; font-size:0.9em;'>---</div>
<div id='nivelLlamaTexto' style='color:#aaa;'>---</div>
<div class='recomendacion-box'>
<div class='recomendacion-titulo'>🎯 RECOMENDACIÓN</div>
<div class='recomendacion-texto' id='recomendacionLlama'>---</div>
<div class='accion-texto' id='accionLlama'>---</div>
<div class='urgencia-box' id='urgenciaLlama'>Urgencia: ○○○○○</div>
</div>
</div>

<div class='sensor-card' id='mq2Card'>
<h3>💨 SENSOR MQ-2 (GASES)</h3>
<div id='mq2Icono' class='mq2-icono'>🌫️</div>
<div class='mq2-container'><div class='mq2-bg'><div id='mq2Fill' class='mq2-fill' style='height:0%'></div></div></div>
<div class='valor-numero' id='valorMQ2'>--%</div>
<div id='estadoMQ2' style='color:#ffd700; font-size:0.9em;'>---</div>
<div id='nivelMQ2Texto' style='color:#aaa;'>---</div>
<div class='recomendacion-box'>
<div class='recomendacion-titulo'>🎯 RECOMENDACIÓN</div>
<div class='recomendacion-texto' id='recomendacionMQ2'>---</div>
<div class='accion-texto' id='accionMQ2'>---</div>
<div class='urgencia-box' id='urgenciaMQ2'>Urgencia: ○○○○○</div>
</div>
</div>

<div class='sensor-card'>
<h3>🌧️ SENSOR DE LLUVIA - IA DIFUSA</h3>
<div class='gota-container'><div class='gota-bg'><div id='lluviaFill' class='gota-fill' style='height:0%'></div></div></div>
<div class='valor-numero' id='valorLluvia'>--%</div>
<div id='estadoLluvia' style='color:#ffd700; font-size:0.9em;'>---</div>
<div id='nivelLluviaTexto' style='color:#aaa;'>---</div>
<div style='font-size:0.8em; margin:5px 0;'>Tendencia: <span id='tendenciaLluvia'>---</span></div>
<div class='recomendacion-box'>
<div class='recomendacion-titulo'>🎯 RECOMENDACIÓN</div>
<div class='recomendacion-texto' id='recomendacionLluvia'>---</div>
<div class='accion-texto' id='accionLluvia'>---</div>
<div class='urgencia-box' id='urgenciaLluvia'>Urgencia: ○○○○○</div>
</div>
<div style='margin-top:15px; background:#00000066; padding:10px; border-radius:10px;'>
<div style='font-size:0.8em; color:#ffd700;'>LÓGICA DIFUSA</div>
<div style='display:flex; justify-content:space-between; font-size:0.7em;'>
<span>🌞 Seco: <span id='membresiaSeco'>0%</span></span>
<span>⛅ Leve: <span id='membresiaLeve'>0%</span></span>
<span>🌧️ Mod: <span id='membresiaModerado'>0%</span></span>
<span>🌊 Int: <span id='membresiaIntenso'>0%</span></span>
</div>
<div class='progress-bar' style='height:8px; margin-top:5px; background:#222;'><div id='barSeco' class='progress-fill' style='width:0%; background:#ffaa00; height:100%; float:left;'></div><div id='barLeve' class='progress-fill' style='width:0%; background:#ff8800; height:100%; float:left;'></div><div id='barModerado' class='progress-fill' style='width:0%; background:#ff5500; height:100%; float:left;'></div><div id='barIntenso' class='progress-fill' style='width:0%; background:#ff0000; height:100%; float:left;'></div></div>
</div>
</div>

<div class='sensor-card' id='colorCard'>
<h3>🎨 SENSOR DE COLOR TCS34725</h3>
<div id='colorIcono' class='color-icono'>🟢</div>
<div id='colorCircle' class='color-container' style='background:#333'></div>
<div class='valor-numero' id='colorDetectado'>--</div>
<div id='colorRGB' class='color-rgb'>R:0 G:0 B:0</div>
<div id='estadoColor' style='color:#ffd700; font-size:0.9em;'>---</div>
</div>
</div>

<div class='termico-panel' id='termicoPanel'>
<div class='termico-header'>🔥 CÁMARA TÉRMICA AMG8833 - IA DIFUSA PDVSA 🔥</div>
<div class='stats-termico'>
<div class='stat-box'><div>MÁX</div><div class='max' id='tmax'>0.0°C</div></div>
<div class='stat-box'><div>MÍN</div><div class='min' id='tmin'>0.0°C</div></div>
<div class='stat-box'><div>PROM</div><div class='prom' id='tprom'>0.0°C</div></div>
</div>
<div class='termico-grid' id='termico-grid'>
)rawliteral";

  for (int i = 0; i < 64; i++) {
    html += "<div class='termico-pixel' id='pixel-" + String(i) + "'></div>";
  }
  
  html += R"rawliteral(
</div>
<div class='leyenda'>
<div><span class='color-box' style='background:#FF0000;'></span>>50°</div>
<div><span class='color-box' style='background:#FF6600;'></span>45-50°</div>
<div><span class='color-box' style='background:#FFFF00;'></span>40-45°</div>
<div><span class='color-box' style='background:#00FF00;'></span>35-40°</div>
<div><span class='color-box' style='background:#00AAFF;'></span>30-35°</div>
<div><span class='color-box' style='background:#0066FF;'></span>25-30°</div>
<div><span class='color-box' style='background:#0000FF;'></span><25°</div>
</div>

<div class='termico-recomendacion-box'>
<div class='recomendacion-titulo'>📊 DIAGNÓSTICO IA TÉRMICA (PDVSA)</div>
<div class='recomendacion-texto' id='termicoEstado' style='color:#ffaa55; font-size:1em;'>---</div>
<div class='recomendacion-titulo' style='margin-top:12px;'>🎯 RECOMENDACIÓN DE SEGURIDAD</div>
<div class='recomendacion-texto' id='termicoRecomendacion'>---</div>
<div class='recomendacion-titulo' style='margin-top:12px;'>⚡ ACCIÓN RECOMENDADA</div>
<div class='accion-texto' id='termicoAccion'>---</div>
<div class='urgencia-box' id='termicoUrgencia' style='margin-top:10px;'>Urgencia: ○○○○○</div>
<div id='termicoAlerta' class='termico-alerta' style='display:none;'></div>
</div>
</div>

<button class='btn-emergencia' onclick="fetch('/emergencia').then(() => location.reload())">
⛔ PARO DE EMERGENCIA ⛔
</button>

<div class='footer'>
<p>🔗 IP: 192.168.4.1 | Radar Activo | Sensores: DHT11, Lluvia , Llama, LDR, GPS, MQ-2, AMG8833, TCS34725</p>
<p>🇻🇪 CYBERION - GUARDIANS OF THE SOIL - ORC 2026 🏆</p>
</div>
</div>
</div>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}