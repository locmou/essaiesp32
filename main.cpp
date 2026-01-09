#include <Arduino.h>
#include "DHTesp.h" 
#include <LiquidCrystal_I2C.h>

/** Initialize DHT sensor 1 */
DHTesp dhtSensor;

// ---------- CONFIG LCD ----------
LiquidCrystal_I2C lcd(0x27, 20, 4); // Adapter l'adresse I2C (0x27 ou 0x3F)

/** Pin number for DHT11 1 data pin */
const uint8_t dhtPin = 32;

//Définition des contrastes
const uint8_t BRIGHTNESS_PIN=5;   // Must be a PWM pin

// Définir la broche 33 comme entrée analogique
const uint8_t LDR = 33;
uint8_t bright;

// ---------- CONFIG MQ-7 / ESP32 ----------
#define MQ7_PIN 34              // Entrée analogique
#define MESURE_INTERVAL 10000    // ms entre deux mesures + publication
unsigned long lastMeasure = 0;

void Retroeclairage(){
//réglage de l'intensité lumineus du LCD selon la lumière ambiante
  bright=(analogRead(LDR)/4);
  analogWrite(BRIGHTNESS_PIN, bright);
  }

  void setup() {
// 
	Serial.begin(115200);
	Serial.println("Lecture DHT11 + affichage LCD");

// Déclaration des broches
  pinMode(BRIGHTNESS_PIN, OUTPUT);
  pinMode(LDR, INPUT);

// LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

// Initialize temperature sensor 1
  dhtSensor.setup(dhtPin, DHTesp::DHT11);
} 

void loop() {
// réglage led rétroéclairage
  Retroeclairage();

// Lecture des données du capteur
  TempAndHumidity data = dhtSensor.getTempAndHumidity();

// Affichage sur le moniteur série
  Serial.print("Température: ");
  Serial.print(data.temperature, 1);
  Serial.print("°C, Humidité: ");
  Serial.print(data.humidity, 1);
  Serial.println("%");

// Affichage sur le LCD
  lcd.setCursor(0, 0);  lcd.print("T°: ");  lcd.print(data.temperature, 1);  lcd.print(" C");
  lcd.setCursor(10, 0);  lcd.print("Hum: ");  lcd.print(data.humidity, 1);  lcd.print(" %");

// Affichage bright 
  lcd.setCursor(0, 1);  lcd.print("Bright: ");  lcd.print(bright);

// Attendre 30 secondes avant la prochaine lecture
  delay(2000);

// Lecture MQ7
  unsigned long now = millis();
    if (now - lastMeasure > MESURE_INTERVAL) {
     lastMeasure = now;

    // Lecture brute du MQ-7 (0-4095 sur ESP32)
    int rawValue = analogRead(MQ7_PIN);

    // Conversion approximative en "pseudo ppm"
    float pseudoPPM = (rawValue / 4095.0) * 1000.0;

// Affichage série
    /*Serial.print("MQ7 brut = ");
    Serial.print(rawValue);
    Serial.print("  ~");
    Serial.print(pseudoPPM);
    Serial.println(" ppm (approx)");*/

    // Affichage LCD 20x4
    lcd.setCursor(0, 2);
    lcd.print("CO MQ7 =>  "); lcd.print("Brut : "); lcd.print(rawValue);

    lcd.setCursor(0, 3);
    lcd.print("Approx : ");
    lcd.print((int)pseudoPPM);
    lcd.print(" ppm");

/*
    lcd.setCursor(0, 3);
    lcd.print("WiFi: ");
    lcd.print(WiFi.SSID());
*/
/*
    // Publication MQTT (envoi de la valeur brute)
    char payload[32];
    snprintf(payload, sizeof(payload), "%d", rawValue);
    client.publish(mqtt_topic, payload);
*/

  }

}