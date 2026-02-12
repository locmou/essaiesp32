#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Mounwiff";
const char* pass = "rue_de_la_Grande680Plage_10!";

const char* mqtt_server = "192.168.1.11";
const char* mqtt_user = "loic.mounier@laposte.net";
const char* mqtt_pass = "vgo:?2258H";

WiFiClient espClient;
PubSubClient client(espClient);

void reconnect() {
  while (!client.connected()) {
    if (client.connect("stationair", mqtt_user, mqtt_pass,
                       "stationair/status", 0, true, "offline")) {

      client.publish("stationair/status","online",true);

      client.publish(
        "homeassistant/sensor/stationair_temp/config",
        "{\"name\":\"Temp\",\"uniq_id\":\"stationair_temp_v2\",\"stat_t\":\"stationair/temp\",\"avty_t\":\"stationair/status\",\"dev_cla\":\"temperature\",\"unit_of_meas\":\"°C\",\"device\":{\"ids\":[\"stationair_v2\"],\"name\":\"Station Air\"}}",
        true
      );

      client.publish("stationair/temp","22.0",true);
    }
  }
}

void setup() {

  Serial.begin(115200);

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  client.setServer(mqtt_server,1883);
  reconnect();
}

void loop() {

  if (!client.connected()) reconnect();

  client.loop();

  static unsigned long last=0;

  if(millis()-last>2000){
    last=millis();
    Serial.println("alive");
    client.publish("stationair/temp","22.5",false);
  }
}
