#include <Arduino.h>

#include <Ticker.h>

#include "DHTesp.h" // Click here to get the library: http://librarymanager/All#DHTesp
#include <LiquidCrystal_I2C.h>
#ifndef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP32 ONLY!)
#error Select ESP32 board.
#endif

/** Initialize DHT sensor 1 */
DHTesp dhtSensor1;

// ---------- CONFIG LCD ----------
LiquidCrystal_I2C lcd(0x27, 20, 4); // Adapter l'adresse I2C (0x27 ou 0x3F)

// Définir la broche 33 comme entrée analogique
const int photoresistorPin = 33;

/** Task handle for the light value read task */
TaskHandle_t tempTaskHandle = NULL;
/** Pin number for DHT11 1 data pin */
int dhtPin1 = 34;

/** Ticker for temperature reading */
Ticker tempTicker;
/** Flags for temperature readings finished */
bool gotNewTemperature = false;
/** Data from sensor 1 */
TempAndHumidity sensor1Data;


/* Flag if main loop is running */
bool tasksEnabled = false;



/**
 * Task to reads temperature from DHT11 sensor
 * @param pvParameters
 *		pointer to task parameters
 */
void tempTask(void *pvParameters) {
	Serial.println("tempTask loop started");
	while (1) // tempTask loop
	{
		if (tasksEnabled && !gotNewTemperature) { // Read temperature only if old data was processed already
			// Reading temperature for humidity takes about 250 milliseconds!
			// Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
			sensor1Data = dhtSensor1.getTempAndHumidity();	// Read values from sensor 1

			gotNewTemperature = true;
		}
		vTaskSuspend(NULL);
	}
}

/**
 * triggerGetTemp
 * Sets flag dhtUpdated to true for handling in loop()
 * called by Ticker tempTicker
 */
void triggerGetTemp() {
	if (tempTaskHandle != NULL) {
		 xTaskResumeFromISR(tempTaskHandle);
	}
}

/**
 * Arduino setup function (called once after boot/reboot)
 */
void setup() {
	Serial.begin(115200);
	Serial.println("Example for 3 DHT11/22 sensors");

	  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
	// Initialize temperature sensor 1
	dhtSensor1.setup(dhtPin1, DHTesp::DHT11);


	// Start task to get temperature
	xTaskCreatePinnedToCore(
			tempTask,											 /* Function to implement the task */
			"tempTask ",										/* Name of the task */
			4000,													 /* Stack size in words */
			NULL,													 /* Task input parameter */
			5,															/* Priority of the task */
			&tempTaskHandle,								/* Task handle. */
			1);														 /* Core where the task should run */

	if (tempTaskHandle == NULL) {
		Serial.println("[ERROR] Failed to start task for temperature update");
	} else {
		// Start update of environment data every 30 seconds
		tempTicker.attach(30, triggerGetTemp);
	}

	// Signal end of setup() to tasks
	tasksEnabled = true;
} // End of setup.


/**
 * loop
 * Arduino loop function, called once 'setup' is complete (your own code
 * should go here)
 */
void loop() {
	if (gotNewTemperature) {
		Serial.print("Sensor data: ");
		Serial.println("Temp: " + String(sensor1Data.temperature,2) + "'C Humidity: " + String(sensor1Data.humidity,1) + "%");
   		
		// Affichage LCD 20x4
    	lcd.clear();
    	lcd.setCursor(0, 0);
    	lcd.print("Temp: " + String(sensor1Data.temperature,2) + "'C Humidity: " + String(sensor1Data.humidity,1) + "%");

		gotNewTemperature = false;
	}

	  int lumi = analogRead(photoresistorPin);
	  Serial.println(lumi);
  // Attendre un peu avant la prochaine lecture
  delay(1000); // 1 seconde
  
} // End of loop