// emisor :
    #include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // <--- IMPORTANTE

uint8_t receptorAdress[] = {0x68, 0x25, 0xDD, 0x2F, 0xCA, 0x9C};

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // FORZAR CANAL 1
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) return;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receptorAdress, 6);
  peer.channel = 1; // <--- DEBE SER 1
  peer.encrypt = false;
  
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Error al añadir peer");
    return;
  }
}

void loop() {
  uint8_t msg = 1;
  // Intentar enviar
  esp_err_t result = esp_now_send(receptorAdress, &msg, sizeof(msg));
  
  if (result == ESP_OK) {
    Serial.println("Intento de envío exitoso");
  } else {
    Serial.println("Error en el envío");
  }
  delay(1000); // Un segundo para no saturar el monitor
}

// receptor : 
    
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // <--- IMPORTANTE

volatile bool mensaje_recibido = false; // volatile para callbacks

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  
  // FORZAR CANAL 1 (Igual que el emisor)
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.print("MI MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) return;

  // Callback ajustado para Core 3.x
  esp_now_register_recv_cb([](const esp_now_recv_info *info, const uint8_t *data, int len) {
    if (len == 1 && data[0] == 1) {
      mensaje_recibido = true;
    }
  });

  Serial.println("Receptor listo en Canal 1...");
}

void loop() {
  if (mensaje_recibido) {
    Serial.println("¡Mensaje recibido correctamente! (Dato: 1)");
    mensaje_recibido = false; // RESETEAR la bandera para ver el siguiente
  }
}