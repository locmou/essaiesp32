/*
Station Air - ESP32
Avec contrôle MQTT bidirectionnel depuis Home Assistant
- Switch MODE_T / MODE_P depuis HA
- Possibilité d'extension pour affichage énergie
*/

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#define MQTT_MAX_PACKET_SIZE 1700
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

// ========== OBJETS CAPTEURS ==========
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

// ---------- CONFIG LCD ----------
LiquidCrystal_I2C lcd(0x27, 20, 4);

#define BRIGHTNESS_PIN 5
#define LDR 33
uint8_t bright;

// ---------- CONFIG MQ-7 / ESP32 ----------
#define MQ7_PIN 34
#define CYCLE_3mn 180000
#define RL_VALUE 10.0
#define RO_CLEAN_AIR_FACTOR 27.5
#define ADC_RESOLUTION 4095.0
float Ro = 1.95;
unsigned long last_3m_time = -CYCLE_3mn;
int rawValue;
float rs;
float ratio;
float ppm;
float press_hPa;
float tempture;
float Humite;

// Caractères LCD - Gros chiffres
byte LT[8] = {B00111, B01111, B11111, B11111, B11111, B11111, B11111, B11111};
byte UB[8] = {B11111, B11111, B11111, B00000, B00000, B00000, B00000, B00000};
byte RT[8] = {B11100, B11110, B11111, B11111, B11111, B11111, B11111, B11111};
byte LL[8] = {B11111, B11111, B11111, B11111, B11111, B11111, B01111, B00111};
byte LB[8] = {B00000, B00000, B00000, B00000, B00000, B11111, B11111, B11111};
byte LR[8] = {B11111, B11111, B11111, B11111, B11111, B11111, B11110, B11100};
byte MB[8] = {B11111, B11111, B11111, B00000, B00000, B00000, B11111, B11111};
byte block[8] = {B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111};

// Icônes météo
byte soleil[8][8] = {
  {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F},
  {0x1F, 0x1F, 0x1F, 0x1E, 0x1C, 0x1C, 0x1C, 0x14},
  {0x10, 0x00, 0x10, 0x0E, 0x01, 0x00, 0x00, 0x00},
  {0x0F, 0x00, 0x00, 0x00, 0x10, 0x0E, 0x01, 0x00},
  {0x1F, 0x1E, 0x19, 0x10, 0x10, 0x10, 0x08, 0x08},
  {0x01, 0x00, 0x00, 0x00, 0x10, 0x10, 0x08, 0x04},
  {0x00, 0x10, 0x0C, 0x02, 0x01, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x0C, 0x02}
};

byte nuageSoleil[8][8] = {
  {0x00, 0x00, 0x03, 0x05, 0x0F, 0x0F, 0x0F, 0x0F},
  {0x01, 0x00, 0x1C, 0x1C, 0x1C, 0x17, 0x1F, 0x1F},
  {0x02, 0x02, 0x12, 0x0F, 0x10, 0x18, 0x1C, 0x00},
  {0x04, 0x08, 0x10, 0x00, 0x10, 0x10, 0x17, 0x10},
  {0x1F, 0x16, 0x1B, 0x0F, 0x07, 0x00, 0x00, 0x00},
  {0x1F, 0x17, 0x1B, 0x1F, 0x1F, 0x00, 0x00, 0x00},
  {0x1C, 0x0F, 0x1E, 0x1E, 0x1A, 0x02, 0x02, 0x02},
  {0x10, 0x08, 0x04, 0x02, 0x01, 0x00, 0x00, 0x00}
};

byte nuage[8][8] = {
  {0x00, 0x00, 0x00, 0x01, 0x07, 0x0D, 0x0D, 0x1E},
  {0x00, 0x00, 0x00, 0x18, 0x1E, 0x17, 0x1B, 0x1E},
  {0x00, 0x00, 0x00, 0x00, 0x03, 0x0E, 0x1D, 0x0F},
  {0x00, 0x00, 0x00, 0x00, 0x18, 0x1E, 0x1D, 0x0F},
  {0x1B, 0x1F, 0x0C, 0x07, 0x00, 0x00, 0x00, 0x00},
  {0x1B, 0x1B, 0x1D, 0x1F, 0x00, 0x00, 0x00, 0x00},
  {0x1F, 0x1D, 0x1F, 0x17, 0x00, 0x00, 0x00, 0x00},
  {0x17, 0x1F, 0x1A, 0x1C, 0x00, 0x00, 0x00, 0x00}
};

byte pluie[8][8] = {
  {0x00, 0x00, 0x03, 0x07, 0x0F, 0x0D, 0x1D, 0x1E},
  {0x00, 0x00, 0x1C, 0x1C, 0x1F, 0x17, 0x1B, 0x1F},
  {0x00, 0x00, 0x03, 0x0F, 0x1F, 0x1E, 0x1D, 0x0F},
  {0x00, 0x1C, 0x17, 0x1B, 0x1F, 0x1B, 0x1D, 0x1F},
  {0x1B, 0x1F, 0x1C, 0x0F, 0x09, 0x09, 0x12, 0x12},
  {0x1F, 0x1B, 0x1D, 0x1F, 0x09, 0x09, 0x12, 0x12},
  {0x1F, 0x1D, 0x1F, 0x17, 0x09, 0x09, 0x12, 0x12},
  {0x17, 0x1F, 0x1B, 0x1E, 0x08, 0x08, 0x10, 0x10}
};

byte tropco[8][8] = {
  {0x00, 0x07, 0x0F, 0x1C, 0x18, 0x18, 0x18, 0x18},
  {0x00, 0x1C, 0x1E, 0x07, 0x03, 0x00, 0x00, 0x00},
  {0x00, 0x07, 0x0F, 0x1C, 0x18, 0x18, 0x18, 0x18},
  {0x00, 0x1C, 0x1E, 0x07, 0x03, 0x03, 0x03, 0x03},
  {0x18, 0x18, 0x18, 0x18, 0x1C, 0x0F, 0x07, 0x00},
  {0x00, 0x00, 0x00, 0x03, 0x07, 0x1E, 0x1C, 0x00},
  {0x18, 0x18, 0x18, 0x18, 0x1C, 0x0F, 0x07, 0x00},
  {0x03, 0x03, 0x03, 0x03, 0x07, 0x1E, 0x1C, 0x00}
};

// ========== CONFIG WIFI/MQTT ==========
WiFiMulti wifiMulti;
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_user = MQTT_USER;
const char* mqtt_pass = MQTT_PASS;

WiFiClient espClient;
PubSubClient client(espClient);

// ========== TIMING ==========
unsigned long last_1s_time = 0;
const unsigned long CYCLE_30s = 30000;
unsigned long last_30s_time = -CYCLE_30s;
int mqttReconnectAttempts = 0;
const int MAX_MQTT_ATTEMPTS = 3;
static bool mqttWasDisconnected = false;

// ========== MESSAGES DISCOVERY EN PROGMEM ==========
const char discovery_temp_json[] PROGMEM = R"({
"name":"Temperature",
"uniq_id":"stationair_temp",
"stat_t":"stationair/data",
"avty_t":"stationair/status",
"dev_cla":"temperature",
"unit_of_meas":"°C",
"val_tpl":"{{value_json.temperature}}",
"device":{"ids":["stationair"],"name":"Station Air","mf":"DIY","mdl":"ESP32"}
})";

const char discovery_hum_json[] PROGMEM = R"({
"name":"Humidite",
"uniq_id":"stationair_hum",
"stat_t":"stationair/data",
"avty_t":"stationair/status",
"dev_cla":"humidity",
"unit_of_meas":"%",
"val_tpl":"{{value_json.humidity}}",
"device":{"ids":["stationair"],"name":"Station Air","mf":"DIY","mdl":"ESP32"}
})";

const char discovery_co_json[] PROGMEM = R"({
"name":"CO",
"uniq_id":"stationair_co",
"stat_t":"stationair/data",
"avty_t":"stationair/status",
"unit_of_meas":"ppm",
"val_tpl":"{{value_json.co}}",
"device":{"ids":["stationair"],"name":"Station Air","mf":"DIY","mdl":"ESP32"}
})";

const char discovery_press_json[] PROGMEM = R"({
"name":"Pression",
"uniq_id":"stationair_press",
"stat_t":"stationair/data",
"avty_t":"stationair/status",
"dev_cla":"pressure",
"unit_of_meas":"hPa",
"val_tpl":"{{value_json.pressure}}",
"device":{"ids":["stationair"],"name":"Station Air","mf":"DIY","mdl":"ESP32"}
})";

// ========== NOUVEAU : SWITCH MODE AFFICHAGE ==========
const char discovery_switch_mode[] PROGMEM = R"({
"name":"Mode Affichage",
"uniq_id":"stationair_mode",
"cmd_t":"stationair/mode/set",
"stat_t":"stationair/mode/state",
"payload_on":"pression",
"payload_off":"temperature",
"state_on":"pression",
"state_off":"temperature",
"icon":"mdi:swap-horizontal",
"device":{"ids":["stationair"],"name":"Station Air","mf":"DIY","mdl":"ESP32"}
})";

char mqttBuffer[600];

// ========== MODES AFFICHAGE ==========
enum modeaff {
  MODE_T,
  MODE_P,
};

modeaff aff;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// FONCTIONS ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void Retroeclairage() {
  bright = (analogRead(LDR) / 4);
  analogWrite(BRIGHTNESS_PIN, bright);
}

float readRS(int adcValue) {
  if (adcValue == 0) return 9999.0;
  float voltage = (adcValue / ADC_RESOLUTION) * 3.3;
  if (voltage < 0.01) return 9999.0;
  float rs = ((3.3 * RL_VALUE) / voltage) - RL_VALUE;
  return rs;
}

float calculatePPM(float ratio) {
  float m = -0.46;
  float b = 0.42;
  float ppm = pow(10, ((log10(ratio) - b) / m));
  return ppm;
}

void lcdslotbigdigit() {
  lcd.createChar(0, LT);
  lcd.createChar(1, UB);
  lcd.createChar(2, RT);
  lcd.createChar(3, LL);
  lcd.createChar(4, LB);
  lcd.createChar(5, LR);
  lcd.createChar(6, MB);
  lcd.createChar(7, block);
}

void printBigDigit(int digit, int col, int row) {
  switch(digit) {
    case 0:
      lcd.setCursor(col, row);     lcd.write(0); lcd.write(1); lcd.write(2);
      lcd.setCursor(col, row + 1); lcd.write(3); lcd.write(4); lcd.write(5);
      break;
    case 1:
      lcd.setCursor(col, row);     lcd.write(1); lcd.write(2); lcd.print(" ");
      lcd.setCursor(col, row + 1); lcd.write(4); lcd.write(7); lcd.write(4);
      break;
    case 2:
      lcd.setCursor(col, row);     lcd.write(6); lcd.write(6); lcd.write(2);
      lcd.setCursor(col, row + 1); lcd.write(3); lcd.write(4); lcd.write(4);
      break;
    case 3:
      lcd.setCursor(col, row);     lcd.write(1); lcd.write(6); lcd.write(2);
      lcd.setCursor(col, row + 1); lcd.write(4); lcd.write(6); lcd.write(5);
      break;
    case 4:
      lcd.setCursor(col, row);     lcd.write(3); lcd.write(4); lcd.write(7);
      lcd.setCursor(col, row + 1); lcd.print(" "); lcd.print(" "); lcd.write(7);
      break;
    case 5:
      lcd.setCursor(col, row);     lcd.write(0); lcd.write(6); lcd.write(6);
      lcd.setCursor(col, row + 1); lcd.write(4); lcd.write(4); lcd.write(5);
      break;
    case 6:
      lcd.setCursor(col, row);     lcd.write(0); lcd.write(6); lcd.write(6);
      lcd.setCursor(col, row + 1); lcd.write(3); lcd.write(4); lcd.write(5);
      break;
    case 7:
      lcd.setCursor(col, row);     lcd.write(1); lcd.write(1); lcd.write(2);
      lcd.setCursor(col, row + 1); lcd.print(" "); lcd.print(" "); lcd.write(7);
      break;
    case 8:
      lcd.setCursor(col, row);     lcd.write(0); lcd.write(6); lcd.write(2);
      lcd.setCursor(col, row + 1); lcd.write(3); lcd.write(6); lcd.write(5);
      break;
    case 9:
      lcd.setCursor(col, row);     lcd.write(0); lcd.write(6); lcd.write(2);
      lcd.setCursor(col, row + 1); lcd.write(4); lcd.write(4); lcd.write(5);
      break;
  }
}

void clean2prems(int lign) {
  lcd.setCursor(0, lign);
  lcd.print("                    ");
  lcd.setCursor(0, lign+1);
  lcd.print("                    ");
}

void printBigNumber(float number, int col, int lign) {
  int entier = (int)number;
  int decimale = abs((int)((number - entier) * 10));
  clean2prems(lign);
  
  String entierStr = String(entier);
  int numDigitsEntier = entierStr.length();

  for(int i = 0; i < entierStr.length(); i++) {
    int digit = entierStr.charAt(i) - '0';
    printBigDigit(digit, col + (i * 4), lign);
  }

  int pointCol = col + (numDigitsEntier * 4);
  lcd.setCursor(pointCol, lign+1);
  lcd.print(".");
  printBigDigit(decimale, pointCol+1, lign);
}

void lcdslotsoleil() {
  for (int i = 0; i < 8; i++) {
    lcd.createChar(i, soleil[i]);
  }
}

void lcdslotnuagesoleil() {
  for (int i = 0; i < 8; i++) {
    lcd.createChar(i, nuageSoleil[i]);
  }
}

void lcdslotnuage() {
  for (int i = 0; i < 8; i++) {
    lcd.createChar(i, nuage[i]);
  }
}

void lcdslotpluie() {
  for (int i = 0; i < 8; i++) {
    lcd.createChar(i, pluie[i]);
  }
}

void printMeteo(int type, int col, int row) {
  clean2prems(row);
  switch(type) {
    case 0: lcdslotsoleil(); break;
    case 1: lcdslotnuagesoleil(); break;
    case 2: lcdslotnuage(); break;
    case 3: lcdslotpluie(); break;
  }

  lcd.setCursor(col, row);
  lcd.write(0); lcd.write(1); lcd.write(2); lcd.write(3);
  lcd.setCursor(col, 1 + row);
  lcd.write(4); lcd.write(5); lcd.write(6); lcd.write(7);
}

void printco(int col, int row) {
  for (int i = 0; i < 8; i++) {
    lcd.createChar(i, tropco[i]);
  }
  lcd.setCursor(col, row);
  lcd.write(0); lcd.write(1); lcd.write(2); lcd.write(3);
  lcd.setCursor(col, row + 1);
  lcd.write(4); lcd.write(5); lcd.write(6); lcd.write(7);
}

void affichmesures23() {
  lcd.setCursor(0, 2);
  lcd.printf("Tmp: %.1fC Hum:%4.1f%% ",tempture, Humite);
  lcd.setCursor(0, 3);
  
  if (WiFi.status() != WL_CONNECTED) {
    lcd.print("WiFi:OFF ");
  } else if (!client.connected()) {
    lcd.print("MQTT:OFF ");
  } else {
    lcd.printf("CO:%7.4f P:%4.0fhPa  ", ppm, press_hPa);
  }
}

void affichageModeT() {
  lcdslotbigdigit();
  printBigNumber(tempture,4,0);
  lcd.setCursor(16, 1);
  lcd.write(0xDF);
  lcd.print("C");

  lcd.setCursor(0, 2);
  lcd.printf("Hum:%4.1f%%  P:%4.0fhPa",Humite, press_hPa);
  lcd.setCursor(0, 3);
  
  if (WiFi.status() != WL_CONNECTED) {
    lcd.print("WiFi:OFF ");
  } else if (!client.connected()) {
    lcd.print("MQTT:OFF ");
  } else {
    lcd.printf("CO:%7.4f  Lum:%-3d  ", ppm, bright);
  }
}

void affichageModeP() {
  if (press_hPa>1015) {
    printMeteo(0, 2, 0);
  } else if (press_hPa>1002) {
    printMeteo(1, 2, 0);
  } else if (press_hPa>990) {
    printMeteo(2, 2, 0);
  } else {
    printMeteo(3, 2, 0);
  }
  
  lcd.setCursor(8, 1);
  lcd.printf("%4.0fhPa", press_hPa);
  affichmesures23();
}

void affichageAlerte() {
  printco(2,0);
  lcd.setCursor(10,1);
  lcd.print("En exces!");
  affichmesures23();
}

// ========== CALLBACK MQTT (BIDIRECTIONNEL) ==========
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("\n┌─────────────────────────────");
  Serial.print("│ MQTT reçu: ");
  Serial.println(topic);
  
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("│ Payload: ");
  Serial.println(message);
  Serial.println("└─────────────────────────────");
  
  // Switch mode affichage
  if (strcmp(topic, "stationair/mode/set") == 0) {
    if (message == "temperature") {
      Serial.println("➤ MODE: Température");
      aff = MODE_T;
      client.publish("stationair/mode/state", "temperature", true);
    } else if (message == "pression") {
      Serial.println("➤ MODE: Pression");
      aff = MODE_P;
      client.publish("stationair/mode/state", "pression", true);
    }
    
    // Forcer un rafraîchissement immédiat
    last_30s_time = millis() - CYCLE_30s;
  }
}

void setup_wifi() {
  Serial.println("=== Connexion WiFi ===");
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID_1, WIFI_PASS_1);

  Serial.print("Connexion en cours");
  int attempts = 0;
  while (wifiMulti.run() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connecté !");
    Serial.print("IP : ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nÉchec WiFi !");
  }
}

void reconnect_mqtt() {
  mqttReconnectAttempts++;
  Serial.print("Tentative MQTT ");
  Serial.print(mqttReconnectAttempts);
  Serial.print("/");
  Serial.print(MAX_MQTT_ATTEMPTS);
  Serial.print("...");

  if (client.connect(DEVICE_NAME, mqtt_user, mqtt_pass,
                     "stationair/status", 0, true, "offline")) {
    
    Serial.println(" OK !");
    mqttReconnectAttempts = 0;
    
    client.publish("stationair/status", "online", true);
    client.loop();
    delay(100);
    
    Serial.println("Discovery...");
    
    // Temperature
    memset(mqttBuffer, 0, sizeof(mqttBuffer));
    strcpy_P(mqttBuffer, discovery_temp_json);
    if (client.beginPublish("homeassistant/sensor/stationair_temp/config", strlen(mqttBuffer), true)) {
      client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
      client.endPublish();
    }
    client.loop();
    delay(500);
    
    // Humidity
    memset(mqttBuffer, 0, sizeof(mqttBuffer));
    strcpy_P(mqttBuffer, discovery_hum_json);
    if (client.beginPublish("homeassistant/sensor/stationair_hum/config", strlen(mqttBuffer), true)) {
      client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
      client.endPublish();
    }
    client.loop();
    delay(500);
    
    // CO
    memset(mqttBuffer, 0, sizeof(mqttBuffer));
    strcpy_P(mqttBuffer, discovery_co_json);
    if (client.beginPublish("homeassistant/sensor/stationair_co/config", strlen(mqttBuffer), true)) {
      client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
      client.endPublish();
    }
    client.loop();
    delay(500);
    
    // Pressure
    memset(mqttBuffer, 0, sizeof(mqttBuffer));
    strcpy_P(mqttBuffer, discovery_press_json);
    if (client.beginPublish("homeassistant/sensor/stationair_press/config", strlen(mqttBuffer), true)) {
      client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
      client.endPublish();
    }
    client.loop();
    delay(500);
    
    // ========== NOUVEAU : Switch Mode Affichage ==========
    memset(mqttBuffer, 0, sizeof(mqttBuffer));
    strcpy_P(mqttBuffer, discovery_switch_mode);
    if (client.beginPublish("homeassistant/switch/stationair_mode/config", strlen(mqttBuffer), true)) {
      client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
      client.endPublish();
    }
    client.loop();
    delay(500);
    
    Serial.println("Discovery OK");
    
    // Publier l'état actuel du mode
    const char* current_mode = (aff == MODE_T) ? "temperature" : "pression";
    client.publish("stationair/mode/state", current_mode, true);
    
    // ========== SOUSCRIPTION AUX TOPICS ==========
    Serial.println("=== Souscription ===");
    if (client.subscribe("stationair/mode/set")) {
      Serial.println("✓ stationair/mode/set");
    }
    Serial.println("====================");
    
    client.publish("stationair/data",
                   "{\"temperature\":0,\"humidity\":0,\"co\":0,\"pressure\":0}",
                   true);
    
  } else {
    Serial.print(" Échec (");
    Serial.print(client.state());
    Serial.println(")");
    
    if (mqttReconnectAttempts >= MAX_MQTT_ATTEMPTS) {
      Serial.println("Trop d'échecs MQTT");
      mqttReconnectAttempts = 0;
    }
  }
}

void setup() {
  disableCore0WDT();
  Serial.begin(115200);
  delay(1000);

  setup_wifi();

  // Configuration MQTT
  client.setBufferSize(1700);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);  // ← IMPORTANT : Callback pour bidirectionnel
  client.setKeepAlive(60);
  client.setSocketTimeout(5);

  // Pins
  pinMode(BRIGHTNESS_PIN, OUTPUT);
  pinMode(LDR, INPUT);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  aff = MODE_P;

  // I2C
  Wire.begin(21, 22);
  Serial.println("Init capteurs I2C...");

  // AHT20
  if (!aht.begin()) {
    Serial.println("ERREUR: AHT20 non détecté !");
    while (1) delay(10);
  }
  Serial.println("AHT20 OK");

  // BMP280
  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    Serial.println("ERREUR: BMP280 non détecté !");
    while (1) delay(10);
  }
  Serial.println("BMP280 OK");

  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500
  );

  // Première lecture
  sensors_event_t humid, tempAHT;
  aht.getEvent(&humid, &tempAHT);
  press_hPa = bmp.readPressure() / 100.0F;
  tempture = tempAHT.temperature;
  Humite = humid.relative_humidity;
  
  rawValue = analogRead(MQ7_PIN);
  rs = readRS(rawValue);
  ratio = rs / Ro;
  ppm = calculatePPM(ratio);
  
  Serial.println("Setup terminé !");
}

void loop() {
  unsigned long now = millis();

  // ===== CLIENT.LOOP() TOUTES LES SECONDES =====
  if (now - last_1s_time >= 1000) {
    last_1s_time = now;
    if (client.connected()) {
      client.loop();  // ← IMPORTANT : Traite les messages MQTT entrants
    }
    Retroeclairage();
    Serial.printf("Bright: %i |T: %.1f°C | H: %.1f%% | P: %.0fhPa | CO: %.6fppm\n",
                  bright, tempture, Humite, press_hPa, ppm);
  }

  // ===== CAPTEURS + AFFICHAGE TOUTES LES 30s =====
  if (now - last_30s_time >= CYCLE_30s) {
    last_30s_time = now;

    // Lecture capteurs
    sensors_event_t humid, tempAHT;
    aht.getEvent(&humid, &tempAHT);
    press_hPa = bmp.readPressure() / 100.0F;
    tempture = tempAHT.temperature;
    Humite = humid.relative_humidity;
    
    rawValue = analogRead(MQ7_PIN);
    rs = readRS(rawValue);
    ratio = rs / Ro;
    ppm = calculatePPM(ratio);

    Serial.println("===== Nouvelles mesures =====");

    // Affichage LCD
    if (ppm >= 10) {
      affichageAlerte();
    } else {
      if (aff == MODE_T) affichageModeT();
      else affichageModeP();
    }

    // Vérifier WiFi
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi perdu");
      lcd.setCursor(0, 3);
      lcd.print("WiFi OFF - Reconnex");
      setup_wifi();
      
      if (WiFi.status() == WL_CONNECTED) {
        mqttReconnectAttempts = 0;
      }
    }
    
    // Vérifier MQTT
    if (WiFi.status() == WL_CONNECTED && !client.connected()) {
      Serial.println("MQTT déconnecté");
      lcd.setCursor(0, 3);
      lcd.print("MQTT OFF - Reconnex");
      mqttWasDisconnected = true;
      reconnect_mqtt();
    }

    // Effacer message si reconnexion
    if (mqttWasDisconnected && client.connected()) {
      lcd.setCursor(0, 3);
      lcd.print("MQTT OK            ");
      mqttWasDisconnected = false;
      delay(1000);
      lcd.setCursor(0, 3);
      lcd.print("                   ");
    }
  }

  // ===== PUBLICATION MQTT (toutes les 3 minutes) =====
  if (now - last_3m_time > CYCLE_3mn) {
    last_3m_time = now;
    
    if (client.connected()) {
      client.publish("stationair/status", "online", true);
      
      JsonDocument doc;
      doc["temperature"] = tempture;
      doc["humidity"] = Humite;
      doc["pressure"] = press_hPa;
      doc["co"] = ppm;

      char buffer[256];
      serializeJson(doc, buffer);
      
      if (client.publish("stationair/data", buffer, true)) {
        Serial.println("MQTT: Données publiées");
      } else {
        Serial.println("MQTT: Échec publication");
      }
    } else {
      Serial.println("MQTT: Non connecté");
    }
  }

  delay(10);
}
