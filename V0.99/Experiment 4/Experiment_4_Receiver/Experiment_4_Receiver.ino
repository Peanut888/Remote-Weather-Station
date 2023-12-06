/*
Title: Weather Station Receiver Experiment 4
Programmer: Justin Lam
Date: 15/02/23
Description: Experiment to test RTC and receiving temperature, humidity, pressure, wind speed, VOC index and if it is raining.
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const int CEPin = 7; // nRF24 chip enable pin.
const int CSNPin = 8; // nRF24 chip select pin.
const byte address[6] = "00001"; // Define the address through which the radios will use to communicate.

struct dataPackage { // Declaration and definition of a struct that contains the data.
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  float rpm = 0.0;
  int index = 0;
  int rain = 1;
};

RF24 radio(CEPin, CSNPin); // Declare and initialise the radio where CE and CSN are digital pins 7 and 8 respectively.
dataPackage data; // Declare the data struct named "data".

void setup() {
  Serial.begin(9600); // Initialise serial communication.

  // Radio Intialisation
  if (!radio.begin()) { // Check if the module is connected.
    Serial.println("Error: nRF24 radio not responding!");
    Serial.println("Please check wiring and code...");
    while (1);
  }
  else {
    radio.openReadingPipe(0, address); // Set the address for receiving.
    radio.setPALevel(RF24_PA_LOW); // Set the amplification of the radio to low.
    radio.startListening(); // Start receiving.
    Serial.println("nRF24 Initialised!");
  }
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
    Serial.print("Pa | VOC Index: ");
    Serial.print(data.index);
    Serial.print(" | Wind Speed: ");
    Serial.print(data.rpm);
    Serial.print(" RPM | Rain: ");
    
    if (data.rain == 0) { // Print yes of rain sensor returns a logic low.
      Serial.println("Yes");
    }
    else { // Print no if rain sensor returns a logic high.
      Serial.println("No");
    }
  }
}
