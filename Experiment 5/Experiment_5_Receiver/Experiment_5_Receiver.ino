/*
Title: Weather Station Receiver Experiment 5
Programmer: Justin Lam
Date: 21/02/23
Description: Experiment to test RTC, payload acknowledgement and receiving temperature, humidity, pressure, wind speed, VOC index and if it is raining.
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RTClib.h>

const int CEPin = 7; // nRF24 chip enable pin.
const int CSNPin = 8; // nRF24 chip select pin.
const byte address[6] = "00001"; // Define the address through which the radios will use to communicate.
char days[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String timeDate = ""; // String to store time and date.
bool ackSuccess = false; // Status of the acknowledgement transmission.

struct dataPackage { // Declaration and definition of a struct that contains the data.
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  float altitude = 0.0;
  float rpm = 0.0;
  int index = 0;
  int rain = 1;
};

struct acknowledge { // Declaration and definition of a struct that contains the acknowledge data.
  bool isAck = false;    
};

RF24 radio(CEPin, CSNPin); // Declare and initialise the radio where CE and CSN are digital pins 7 and 8 respectively.
RTC_DS3231 clock; // Declare and initialise the DS3231 setting I2C address to 0x68.
dataPackage data; // Declare the data struct named "data".
DateTime now; // Declare and initialise data object.
acknowledge received; // Declare and intitialise acknowledge struct named "received".

void setup() {
  Serial.begin(9600); // Initialise serial communication.

  #ifndef ESP8266
  while (!Serial) { // Check if the serial port has connected.
    Serial.println("Error: failed to start serial communication!");
    delay(3000);
  }
  #endif

  // Radio Initialisation
  if (!radio.begin()) { // Check if the module is connected.
    Serial.println("Error: nRF24 radio not responding!");
    Serial.println("Please check wiring and code...");
    while (1);
  }
  else {
    radio.openReadingPipe(1, address); // Set the address for receiving.
    radio.setPALevel(RF24_PA_LOW); // Set the amplification of the radio to low.
    radio.enableAckPayload(); // Enable the acknowledgement payload.
    radio.writeAckPayload(1, &received, sizeof(acknowledge)); // Send acknowledge packet when data received.
    radio.printDetails();
    radio.startListening(); // Start receiving.
    Serial.println("nRF24 Initialised!");
  }

  // DS3231 Initialisation
  while (!clock.begin()) { // Check if the module is connected.
    Serial.println("Error: DS3231 not detected!");
    Serial.flush();
    delay(1000);
  }

  if (clock.lostPower()) { // Check if the RTC has lost power and reset the time.
    Serial.println("RTC has lost power, resetting the time!");
    clock.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void loop() {
  if (radio.available()) { // Wait for data transmission.
    radio.read(&data, sizeof(dataPackage)); // Read data received.
    received.isAck = true; // The receiver has received a data package.
    ackSuccess = radio.writeAckPayload(1, &received, sizeof(acknowledge)); // Return an acknowledgement package.

    if (ackSuccess) {
      now = clock.now(); // Get the current date and time.
      // Print the time and date data was received on the serial monitor.
      Serial.print("Received at: ");
      timeDate = String(now.hour()) + ':' + String(now.minute()) + ':' + String(now.second()) + ' ' + String(now.day())
        + '/' + String(now.month()) + '/' + String(now.year()) + ' ' + days[now.dayOfTheWeek()];
      Serial.println(timeDate);
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
      Serial.print(" | Wind Speed: ");
      Serial.print(data.rpm);
      Serial.print(" RPM | ");

      if (data.rain == 0) { // Print yes if rain sensor returns a logic low.
        Serial.println("Rain: Yes");
      }
      else { // Print no if rain sensor returns a logic high.
        Serial.println("Rain: No");
      }
      
      delay(250);
    }
    else { // Acknowledgement package failed.
      Serial.println("Error: Payload acknowledgement failed!");
    }

    received.isAck = false; // Reset the acknowledgement status.
  }
}
