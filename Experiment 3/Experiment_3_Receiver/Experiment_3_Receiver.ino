/*
Title: Weather Station Receiver Experiment 3
Programmer: Justin Lam
Date: 13/02/23
Description: Experiment to test RTC and transmitting temperature, humidity, pressure, altitude, VOC index and if it is raining.
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const byte address[6] = "00001"; // Define the address through which the radios will use to communicate.

struct dataPackage { // Declaration and definition of a struct that contains the data.
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  float altitude = 0.0;
  int index = 0;
  int rain = 1;
};

RF24 radio(7, 8); // Declare and initialise the radio where CE and CSN are digital pins 7 and 8 respectively.
dataPackage data; // Declare the data struct named "data".

void setup() {
  Serial.begin(9600);
  radio.begin(); // Start radio operation.
  //radio.setDataRate(RF24_250KBPS); // Set the data rate to 250kb/s.
  //radio.setChannel(120); // Set the channel frequency to channel 120 (2520MHz).
  radio.openReadingPipe(0, address); // Set the address for receiving.
  radio.setPALevel(RF24_PA_LOW); // Set the amplification of the radio to low.
  radio.startListening(); // Start receiving.
  Serial.println("Starting...");
}

void loop() {
  if (radio.available()) { // Check if receiving data.
    radio.read(&data, sizeof(dataPackage)); // Read data received.
    // Print the data on the serial monitor.
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

    if (data.rain == 0) {
      Serial.println(" | Rain: Yes"); // Print yes of rain sensor returns a logic low.
    }
    else {
      Serial.println(" | Rain: No"); // Print no if rain sensor returns a logic high.
    }
  }
}
