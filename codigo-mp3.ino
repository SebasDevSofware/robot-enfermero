#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

// Usamos el segundo puerto serie del ESP32 (UART2)
// Pin 16 es RX2 (conecta al TX del módulo)
// Pin 17 es TX2 (conecta al RX del módulo pasando por la resistencia de 1k)
HardwareSerial myHardwareSerial(2); 
DFRobotDFPlayerMini myDFPlayer;

void setup() {
  // Monitor Serial para depuración
  Serial.begin(115200);
  
  // Inicialización de UART2: 9600 baudios, configuración estándar, RX=16, TX=17
  myHardwareSerial.begin(9600, SERIAL_8N1, 16, 17);

  Serial.println(F("Iniciando comunicación con el módulo MP3..."));

  // Intentar conectar con el módulo
  if (!myDFPlayer.begin(myHardwareSerial)) {
    Serial.println(F("Error: Revisa conexiones, tarjeta SD o alimentación."));
    while(true); // Se detiene si hay error
  }
  
  Serial.println(F("Módulo listo."));

  // Configurar volumen inicial (rango 0 a 30)
  myDFPlayer.volume(25); 

  // --- CAMBIO CLAVE AQUÍ ---
  // playMp3Folder(1) busca específicamente en la carpeta llamada "mp3" 
  // el archivo que comienza con "0001" (ej: 0001.mp3 o 0001_audio.mp3)
  Serial.println(F("Reproduciendo mp3/0001.mp3..."));
  myDFPlayer.playMp3Folder(3); 
}

void loop() {
  // El loop queda libre para otras tareas de tu proyecto
  // Puedes usar myDFPlayer.available() para detectar si terminó de sonar
  if (myDFPlayer.available()) {
    uint8_t type = myDFPlayer.readType();
    int value = myDFPlayer.read();
    
    if (type == DFPlayerPlayFinished) {
       Serial.print(F("El archivo número "));
       Serial.print(value);
       Serial.println(F(" ha terminado de reproducirse."));
    }
  }
}
