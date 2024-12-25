#include "DHT.h"

#define DHTPIN 7          // Define the pin where the DHT sensor is connected
#define DHTTYPE DHT11     // Define the type of DHT sensor: DHT11 or DHT22

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  dht.begin();
  
  Serial.println("DHT Sensor Test");
}

void loop() {
  // Read temperature and humidity from DHT sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  // // Check if any reads failed and exit early (to try again).
  // if (isnan(temperature) || isnan(humidity)) {
  //   Serial.println("Failed to read from DHT sensor!");
  //   delay(2000);
  //   return;
  // }

  // Print the results to the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");

  // Wait for 2 seconds before the next reading
  delay(2000);
}