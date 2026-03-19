/*
 * PROGRAMME EXPÉRIMENTAL : Communication MQTT bidirectionnelle
 * ESP32 ↔ Home Assistant
 * 
 * Fonctionnalités :
 * - ESP32 envoie des données vers HA (température, etc.)
 * - HA envoie des commandes vers ESP32
 * - Variable de test modifiable depuis HA
 * - Affichage sur Serial Monitor
 */

#include <WiFi.h>
#include <WiFiMulti.h>
#define MQTT_MAX_PACKET_SIZE 1700
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ========== CONFIG WIFI ==========
WiFiMulti wifiMulti;

// ========== CONFIG MQTT ==========
const char* mqtt_server = "192.168.1.11";
const int mqtt_port = 1883;
const char* mqtt_user = "loic.mounier@laposte.net";
const char* mqtt_pass = "vgo:?2258H";

WiFiClient espClient;
PubSubClient client(espClient);

// ========== TOPICS MQTT ==========
// Topics de publication (ESP32 → HA)
const char* TOPIC_STATUS = "stationair/status";
const char* TOPIC_DATA = "stationair/data";

// Topics de souscription (HA → ESP32)
const char* TOPIC_COMMAND = "stationair/command";        // Commandes générales
const char* TOPIC_SET_VALUE = "stationair/set_value";    // Modifier la variable test

// ========== VARIABLES DE TEST ==========
int testValue = 0;              // Variable modifiable depuis HA
String lastCommand = "aucune";  // Dernière commande reçue
bool ledState = false;          // État LED (si vous voulez en piloter une)

// ========== TIMING ==========
unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL = 5000;  // Publier toutes les 5 secondes

// ========== FONCTIONS ==========

void setup_wifi() {
  Serial.println("\n=== Connexion WiFi ===");
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP("Mounwiff", "rue_de_la_Grande680Plage_10!");

  Serial.print("Connexion en cours");
  int attempts = 0;
  while (wifiMulti.run() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connecté !");
    Serial.print("IP : ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n✗ Échec WiFi !");
  }
}

// Callback appelé quand un message MQTT est reçu
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("\n┌─────────────────────────────────");
  Serial.print("│ MESSAGE REÇU sur topic : ");
  Serial.println(topic);
  Serial.print("│ Longueur : ");
  Serial.println(length);
  
  // Convertir le payload en String
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("│ Contenu : ");
  Serial.println(message);
  Serial.println("└─────────────────────────────────");
  
  // ===== TRAITEMENT DES COMMANDES =====
  
  // Topic : stationair/command
  if (strcmp(topic, TOPIC_COMMAND) == 0) {
    lastCommand = message;
    
    if (message == "reset") {
      Serial.println("➤ COMMANDE : Reset de la variable test");
      testValue = 0;
    }
    else if (message == "increment") {
      Serial.println("➤ COMMANDE : Incrément +1");
      testValue++;
    }
    else if (message == "decrement") {
      Serial.println("➤ COMMANDE : Décrément -1");
      testValue--;
    }
    else if (message == "led_on") {
      Serial.println("➤ COMMANDE : LED ON");
      ledState = true;
      // digitalWrite(LED_PIN, HIGH);  // Si vous avez une LED
    }
    else if (message == "led_off") {
      Serial.println("➤ COMMANDE : LED OFF");
      ledState = false;
      // digitalWrite(LED_PIN, LOW);
    }
    else {
      Serial.print("➤ COMMANDE INCONNUE : ");
      Serial.println(message);
    }
  }
  
  // Topic : stationair/set_value
  else if (strcmp(topic, TOPIC_SET_VALUE) == 0) {
    int newValue = message.toInt();
    Serial.print("➤ MODIFICATION : testValue = ");
    Serial.println(newValue);
    testValue = newValue;
  }
  
  // Afficher l'état actuel
  Serial.println("\n╔═══════════════════════════════╗");
  Serial.print("║ testValue = ");
  Serial.println(testValue);
  Serial.print("║ lastCommand = ");
  Serial.println(lastCommand);
  Serial.print("║ ledState = ");
  Serial.println(ledState ? "ON" : "OFF");
  Serial.println("╚═══════════════════════════════╝\n");
}

void reconnect_mqtt() {
  Serial.print("Tentative MQTT... ");
  
  if (client.connect("stationair_experimental", mqtt_user, mqtt_pass,
                     TOPIC_STATUS, 0, true, "offline")) {
    
    Serial.println("✓ OK !");
    
    // Publier le statut online
    client.publish(TOPIC_STATUS, "online", true);
    
    // S'ABONNER aux topics (pour recevoir des commandes de HA)
    Serial.println("\n=== SOUSCRIPTION AUX TOPICS ===");
    
    if (client.subscribe(TOPIC_COMMAND)) {
      Serial.print("✓ Abonné à : ");
      Serial.println(TOPIC_COMMAND);
    }
    
    if (client.subscribe(TOPIC_SET_VALUE)) {
      Serial.print("✓ Abonné à : ");
      Serial.println(TOPIC_SET_VALUE);
    }
    
    Serial.println("===============================\n");
    
    // Publier la configuration MQTT Discovery pour HA
    publish_discovery();
    
  } else {
    Serial.print("✗ Échec (code ");
    Serial.print(client.state());
    Serial.println(")");
  }
}

void publish_discovery() {
  Serial.println("Publication MQTT Discovery...");
  
  // Sensor : testValue (nombre affiché dans HA)
  const char* discovery_sensor = R"({
"name":"Test Value",
"uniq_id":"stationair_testvalue",
"stat_t":"stationair/data",
"val_tpl":"{{value_json.testValue}}",
"icon":"mdi:counter",
"device":{"ids":["stationair"],"name":"Station Air Experimental","mf":"DIY","mdl":"ESP32"}
})";
  
  client.publish("homeassistant/sensor/stationair_testvalue/config", discovery_sensor, true);
  delay(100);
  
  // Number : Contrôle de testValue depuis HA (slider)
  const char* discovery_number = R"({
"name":"Set Test Value",
"uniq_id":"stationair_set_testvalue",
"cmd_t":"stationair/set_value",
"stat_t":"stationair/data",
"val_tpl":"{{value_json.testValue}}",
"min":0,
"max":100,
"step":1,
"mode":"slider",
"icon":"mdi:tune",
"device":{"ids":["stationair"],"name":"Station Air Experimental","mf":"DIY","mdl":"ESP32"}
})";
  
  client.publish("homeassistant/number/stationair_set_testvalue/config", discovery_number, true);
  delay(100);
  
  // Button : Reset
  const char* discovery_button_reset = R"({
"name":"Reset Value",
"uniq_id":"stationair_reset",
"cmd_t":"stationair/command",
"payload_press":"reset",
"icon":"mdi:restore",
"device":{"ids":["stationair"],"name":"Station Air Experimental","mf":"DIY","mdl":"ESP32"}
})";
  
  client.publish("homeassistant/button/stationair_reset/config", discovery_button_reset, true);
  delay(100);
  
  // Button : Increment
  const char* discovery_button_inc = R"({
"name":"Increment Value",
"uniq_id":"stationair_increment",
"cmd_t":"stationair/command",
"payload_press":"increment",
"icon":"mdi:plus",
"device":{"ids":["stationair"],"name":"Station Air Experimental","mf":"DIY","mdl":"ESP32"}
})";
  
  client.publish("homeassistant/button/stationair_increment/config", discovery_button_inc, true);
  delay(100);
  
  // Button : Decrement
  const char* discovery_button_dec = R"({
"name":"Decrement Value",
"uniq_id":"stationair_decrement",
"cmd_t":"stationair/command",
"payload_press":"decrement",
"icon":"mdi:minus",
"device":{"ids":["stationair"],"name":"Station Air Experimental","mf":"DIY","mdl":"ESP32"}
})";
  
  client.publish("homeassistant/button/stationair_decrement/config", discovery_button_dec, true);
  
  Serial.println("✓ Discovery terminé\n");
}

void publish_data() {
  if (!client.connected()) {
    Serial.println("⚠ MQTT non connecté, impossible de publier");
    return;
  }
  
  // Créer le JSON avec les données
  JsonDocument doc;
  doc["testValue"] = testValue;
  doc["lastCommand"] = lastCommand;
  doc["ledState"] = ledState;
  doc["uptime"] = millis() / 1000;  // Secondes depuis boot
  
  char buffer[256];
  serializeJson(doc, buffer);
  
  // Publier
  if (client.publish(TOPIC_DATA, buffer, true)) {
    Serial.println("📤 PUBLICATION MQTT :");
    Serial.println(buffer);
  } else {
    Serial.println("✗ Échec publication");
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║  ESP32 ↔ HOME ASSISTANT EXPÉRIMENTAL  ║");
  Serial.println("║      Communication MQTT Bidirectionnelle      ║");
  Serial.println("╚════════════════════════════════════════╝");
  
  // WiFi
  setup_wifi();
  
  // MQTT
  client.setBufferSize(1700);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);  // ← IMPORTANT : définir le callback
  client.setKeepAlive(60);
  client.setSocketTimeout(5);
  
  Serial.println("\n✓ Setup terminé !");
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║ COMMANDES DISPONIBLES DEPUIS HA : ║");
  Serial.println("╠════════════════════════════════════╣");
  Serial.println("║ Topic: stationair/command          ║");
  Serial.println("║   - reset       : testValue = 0    ║");
  Serial.println("║   - increment   : testValue++      ║");
  Serial.println("║   - decrement   : testValue--      ║");
  Serial.println("║   - led_on      : Allumer LED      ║");
  Serial.println("║   - led_off     : Éteindre LED     ║");
  Serial.println("║                                    ║");
  Serial.println("║ Topic: stationair/set_value        ║");
  Serial.println("║   - Envoyer un nombre (0-100)      ║");
  Serial.println("╚════════════════════════════════════╝\n");
}

// ========== LOOP ==========
void loop() {
  unsigned long now = millis();
  
  // Vérifier WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠ WiFi perdu, reconnexion...");
    setup_wifi();
  }
  
  // Vérifier MQTT
  if (!client.connected()) {
    reconnect_mqtt();
    delay(5000);
  }
  
  // TRÈS IMPORTANT : appeler client.loop() pour traiter les messages entrants
  client.loop();
  
  // Publier les données périodiquement
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;
    publish_data();
  }
  
  delay(10);
}
