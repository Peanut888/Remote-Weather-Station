/*
Title: Weather Station Receiver Experiment 2
Programmer: Justin Lam
Date: 11/02/23
Description: Experiment to test receiving temperature, humidity, pressure, altitude and VOC index.
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const byte address[6] = "00001";

struct dataPackage {
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  float altitude = 0.0;
  int index = 0;
};

RF24 radio(7, 8);
dataPackage data;

void setup() {
  Serial.begin(9600);
  radio.begin();
  //radio.setDataRate(RF24_250KBPS);
  //radio.setChannel(120);
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  Serial.println("Starting...");
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(dataPackage));
    Serial.print("Temperature: ");
    Serial.print(data.temperature);
    Serial.print("C | Humidity: ");
    Serial.print(data.humidity);
    Serial.print("% | Pressure: ");
    Serial.print(data.pressure);
    Serial.print("Pa | Altitude: ");
    Serial.print(data.altitude);
    Serial.println("m");
    Serial.print("VOC Index: ");
    Serial.print(data.index);
    Serial.println();
  }
}
