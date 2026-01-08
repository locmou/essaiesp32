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
  lcd.setCursor(0, 0);  lcd.print("Temp: ");  lcd.print(data.temperature, 1);  lcd.print(" C");
  lcd.setCursor(0, 1);  lcd.print("Hum: ");  lcd.print(data.humidity, 1);  lcd.print(" %");

// Lecture luminosité
  int lumi = analogRead(LDR);
  lcd.setCursor(0, 2);  lcd.print("Lum: ");  lcd.print(lumi);

// Attendre 30 secondes avant la prochaine lecture
  delay(2000);

}