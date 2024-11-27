#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <esp_http_client.h>


const char* ssid = "OnePlus 9 5G";
const char* password = "";
const char* apiKey = "03OADTQYBRR6U8FE";  // Replace with your ThingSpeak Write API Key
const char* server = "http://api.thingspeak.com/update";

const int trig_pin = 4;
const int echo_pin = 6;
float timing = 0.0;
float distance = 0.0;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  WiFi.begin(ssid);
}

void sendDataToThingSpeak(String distance) { //put String distance
  esp_http_client_config_t config = {
    .url = server,
  };

  esp_http_client_handle_t client = esp_http_client_init(&config);

  // Prepare the URL
  String url = String(server) + "?api_key=" + apiKey +
               "&field1=" + distance;
             

  // Set URL
  esp_http_client_set_url(client, url.c_str());

  // Make the GET request
  esp_err_t err = esp_http_client_perform(client);

  if (err == ESP_OK) {
    Serial.println("Data sent successfully to ThingSpeak");
    Serial.printf("HTTP Status = %d\n", esp_http_client_get_status_code(client));
  } else {
    Serial.printf("HTTP GET request failed: %s\n", esp_err_to_name(err));
  }

  esp_http_client_cleanup(client);
}

void loop() {
  digitalWrite(trig_pin, LOW);
  delay(2);
  
  digitalWrite(trig_pin, HIGH);
  delay(10);
  digitalWrite(trig_pin, LOW);
  
  timing = pulseIn(echo_pin, HIGH);
  distance = (timing * 0.034) / 2; 
  String Distance = String(distance);
  sendDataToThingSpeak(Distance); 

}
