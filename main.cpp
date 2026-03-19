/*
 * PROGRAMME EXPÉRIMENTAL CORRIGÉ
 * Communication MQTT bidirectionnelle ESP32 ↔ Home Assistant
 * Basé sur le code fonctionnel de votre station
 */

#include <WiFi.h>
#include <WiFiMulti.h>
#define MQTT_MAX_PACKET_SIZE 1700
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ========== CONFIG WIFI/MQTT ==========
WiFiMulti wifiMulti;
const char* mqtt_server = "192.168.1.11";
const int mqtt_port = 1883;
const char* mqtt_user = "loic.mounier@laposte.net";
const char* mqtt_pass = "vgo:?2258H";

WiFiClient espClient;
PubSubClient client(espClient);

// ========== VARIABLES TEST ==========
int testValue = 0;
String lastCommand = "aucune";

// ========== TIMING ==========
unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL = 5000;

// ========== MESSAGES DISCOVERY EN PROGMEM ==========
const char discovery_sensor[] PROGMEM = R"({
"name":"Test Value",
"uniq_id":"stationair_testvalue",
"stat_t":"stationair/data",
"val_tpl":"{{value_json.testValue}}",
"icon":"mdi:counter",
"device":{"ids":["stationair"],"name":"Station Experimental","mf":"DIY","mdl":"ESP32"}
})";

const char discovery_number[] PROGMEM = R"({
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
"device":{"ids":["stationair"],"name":"Station Experimental","mf":"DIY","mdl":"ESP32"}
})";

const char discovery_button_reset[] PROGMEM = R"({
"name":"Reset Value",
"uniq_id":"stationair_reset",
"cmd_t":"stationair/command",
"payload_press":"reset",
"icon":"mdi:restore",
"device":{"ids":["stationair"],"name":"Station Experimental","mf":"DIY","mdl":"ESP32"}
})";

const char discovery_button_inc[] PROGMEM = R"({
"name":"Increment",
"uniq_id":"stationair_increment",
"cmd_t":"stationair/command",
"payload_press":"increment",
"icon":"mdi:plus",
"device":{"ids":["stationair"],"name":"Station Experimental","mf":"DIY","mdl":"ESP32"}
})";

const char discovery_button_dec[] PROGMEM = R"({
"name":"Decrement",
"uniq_id":"stationair_decrement",
"cmd_t":"stationair/command",
"payload_press":"decrement",
"icon":"mdi:minus",
"device":{"ids":["stationair"],"name":"Station Experimental","mf":"DIY","mdl":"ESP32"}
})";

// Buffer global pour les messages MQTT
char mqttBuffer[600];

// ========== CALLBACK MQTT ==========
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("\n┌─────────────────────────────────");
  Serial.print("│ MESSAGE REÇU sur : ");
  Serial.println(topic);
  Serial.print("│ Longueur : ");
  Serial.println(length);
  
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("│ Contenu : ");
  Serial.println(message);
  Serial.println("└─────────────────────────────────");
  
  // Topic : stationair/command
  if (strcmp(topic, "stationair/command") == 0) {
    lastCommand = message;
    
    if (message == "reset") {
      Serial.println("➤ COMMANDE : Reset");
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
    else {
      Serial.print("➤ COMMANDE INCONNUE : ");
      Serial.println(message);
    }
  }
  
  // Topic : stationair/set_value
  else if (strcmp(topic, "stationair/set_value") == 0) {
    int newValue = message.toInt();
    Serial.print("➤ MODIFICATION : testValue = ");
    Serial.println(newValue);
    testValue = newValue;
  }
  
  // Afficher l'état
  Serial.println("\n╔═══════════════════════════════╗");
  Serial.print("║ testValue = ");
  Serial.println(testValue);
  Serial.print("║ lastCommand = ");
  Serial.println(lastCommand);
  Serial.println("╚═══════════════════════════════╝\n");
}

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

void reconnect_mqtt() {
  Serial.print("Tentative MQTT... ");
  
  if (client.connect("stationair_exp", mqtt_user, mqtt_pass,
                     "stationair/status", 0, true, "offline")) {
    
    Serial.println("✓ OK !");
    
    client.publish("stationair/status", "online", true);
    client.loop();
    delay(100);
    
    Serial.println("\n=== MQTT Discovery ===");
    
    // Sensor
    memset(mqttBuffer, 0, sizeof(mqttBuffer));
    strcpy_P(mqttBuffer, discovery_sensor);
    if (client.beginPublish("homeassistant/sensor/stationair_testvalue/config", strlen(mqttBuffer), true)) {
      client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
      client.endPublish();
      Serial.println("✓ Sensor");
    }
    client.loop();
    delay(100);
    
    // Number (slider)
    memset(mqttBuffer, 0, sizeof(mqttBuffer));
    strcpy_P(mqttBuffer, discovery_number);
    if (client.beginPublish("homeassistant/number/stationair_set_testvalue/config", strlen(mqttBuffer), true)) {
      client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
      client.endPublish();
      Serial.println("✓ Number");
    }
    client.loop();
    delay(100);
    
    // Button Reset
    memset(mqttBuffer, 0, sizeof(mqttBuffer));
    strcpy_P(mqttBuffer, discovery_button_reset);
    if (client.beginPublish("homeassistant/button/stationair_reset/config", strlen(mqttBuffer), true)) {
      client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
      client.endPublish();
      Serial.println("✓ Button Reset");
    }
    client.loop();
    delay(100);
    
    // Button Increment
    memset(mqttBuffer, 0, sizeof(mqttBuffer));
    strcpy_P(mqttBuffer, discovery_button_inc);
    if (client.beginPublish("homeassistant/button/stationair_increment/config", strlen(mqttBuffer), true)) {
      client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
      client.endPublish();
      Serial.println("✓ Button Increment");
    }
    client.loop();
    delay(100);
    
    // Button Decrement
    memset(mqttBuffer, 0, sizeof(mqttBuffer));
    strcpy_P(mqttBuffer, discovery_button_dec);
    if (client.beginPublish("homeassistant/button/stationair_decrement/config", strlen(mqttBuffer), true)) {
      client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
      client.endPublish();
      Serial.println("✓ Button Decrement");
    }
    
    Serial.println("=== Discovery terminé ===\n");
    
    // S'ABONNER aux topics
    Serial.println("=== Souscription ===");
    client.subscribe("stationair/command");
    Serial.println("✓ stationair/command");
    client.subscribe("stationair/set_value");
    Serial.println("✓ stationair/set_value");
    Serial.println("====================\n");
    
  } else {
    Serial.print("✗ Échec (");
    Serial.print(client.state());
    Serial.println(")");
  }
}

void publish_data() {
  if (!client.connected()) return;
  
  JsonDocument doc;
  doc["testValue"] = testValue;
  doc["lastCommand"] = lastCommand;
  doc["uptime"] = millis() / 1000;
  
  char buffer[256];
  serializeJson(doc, buffer);
  
  if (client.publish("stationair/data", buffer, true)) {
    Serial.println("📤 Données publiées");
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║   ESP32 ↔ HA EXPERIMENTAL         ║");
  Serial.println("╚════════════════════════════════════╝");
  
  setup_wifi();
  
  client.setBufferSize(1700);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);  // ← IMPORTANT
  client.setKeepAlive(60);
  client.setSocketTimeout(5);
  
  Serial.println("\n✓ Setup terminé !");
}

// ========== LOOP ==========
void loop() {
  unsigned long now = millis();
  
  // WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠ WiFi perdu");
    setup_wifi();
  }
  
  // MQTT
  if (!client.connected()) {
    reconnect_mqtt();
    delay(5000);
  }
  
  // IMPORTANT : traiter les messages entrants
  client.loop();
  
  // Publier périodiquement
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;
    publish_data();
  }
  
  delay(10);
}
