#include <Wire.h>
#include <BME280.h>

// Definitions for BME280 settings for more information on how to use this visit 
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf 

#define PRESSURE_CONFIG 0b00100101
#define HUMIDITY_CONFIG 0b00000001
#define BME_CONFIG      0b00000000

BME280 bme;           // Create sensor object

float sensorData[3];  // Create float array to store sensor data

void setup() {
  Serial.begin(115200);

  Wire.begin();           // Initialize I2C communication
  Wire.setClock(400000);  // Set I2C clock speed to highest permissable by the BME280 sensor

  while(!bme.begin()) {   // Initialize BME280
    Serial.println("BME280 could not be initialized! Check sensor and wiring.");
    delay(1000);
  }

  //bme.setSettings(PRESSURE_CONFIG, HUMIDITY_CONFIG, BME_CONFIG);  // Changes the default settings of the BME280, uncomment to use.
}

void loop() {

  bme.getRaw();               // Reads BME280 data registers
  bme.getData(sensorData, 3); // Calculates pressure, temperature, and humidity from obtained raw data
  
  Serial.print("Temperature: ");
  Serial.print(sensorData[0]);
  Serial.print("C\t");
  Serial.print("Pressure: ");
  Serial.print(sensorData[1]);
  Serial.print("Pa\t");
  Serial.print("Humidity: ");
  Serial.print(sensorData[2]);
  Serial.print("%\n");
  delay(1000);
  
}
