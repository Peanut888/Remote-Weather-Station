/*
Title: Remote Weather Station Slave Receiver V0.99
Programmer: Justin Lam
Date: 01/03/23
Description: Slave receiver for the remote weather station, reads data from the master receiver, records the time at reception and logs the data into an SD card.
*/

#include <Wire.h>
#include <RTClib.h>
#include <SD.h>

const int CSPin = 10; // SD card reader chip select pin.
char days[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String timeDate = ""; // String to store time and date.
String dataStr = ""; // String to store data for writing.
bool hasReceived = false; // Receive status.

struct dataPacket { // Declaration and definition of a struct that contains the data.
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  float altitude = 0.0;
  float rpm = 0.0;
  int indexVOC = 0;
  int indexUV = 0;
  int rain = 1;
};

RTC_DS3231 RTC; // Declare and initialise the DS3231 setting I2C address to 0x68.
dataPacket data; // Declare the data packet named "data".
DateTime now; // Declare and initialise data object.
File dataFile; // File object for writing data.

void setup() {
  Serial.begin(9600); // Initialise serial communications.
  Wire.begin(6); // Initialise I2C communication setting slave Arduino address as 6.

  // Not required
  /*
  #ifndef ESP8266
  while (!Serial) {
    delay(1000);
  }
  #endif
  */

  /* SD Card Reader Initialisation */
  Serial.println("Intitalising SD card...");

  if (!SD.begin(CSPin)) { // Check if the SD card reader is responding.
    Serial.println("Error: SD card not responding!");
  }
  else {
    Serial.println("SD Card Initialised!");
  }

  /* DS3231 Initialisation */
  while (!RTC.begin()) { // Check if the clock module is responding.
    Serial.println("Error: DS3231 not detected!");
    delay(500);
  }

  while (RTC.lostPower()) { // Check if the clock module has lost power and reset the time.
    Serial.println("RTC has lost power, resetting the time!");
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial.println("DS3231 Initialised!");
  Wire.onReceive(receiveEvent); // Setup an I2C receive event.
}

void loop() {
  if (hasReceived) { // Check if data was received on the I2C bus from master Arduino.
    now = RTC.now(); // Get the current time and date.
    dataFile = SD.open("WEATHE~1.CSV", FILE_WRITE); // Open file in SD card for writing.

    if (dataFile) { // Check if file is open.
      // Display the data on the serial monitor.
      Serial.println("Logging Data...");
      timeDate = String(now.hour()) + ':' + String(now.minute()) + ':' + String(now.second()) + ' ' + String(now.day())
        + '/' + String(now.month()) + '/' + String(now.year()) + ' ' + days[now.dayOfTheWeek()]; // Make a time and date string.
      Serial.print("Received at: ");
      Serial.println(timeDate);
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
      Serial.print(data.indexVOC);
      Serial.print(" | UV Index: ");
      Serial.print(data.indexUV);
      Serial.print(" | Wind Speed: ");
      Serial.print(data.rpm);
      Serial.print(" RPM | ");

      if (data.rain == 0) { // Print yes if rain sensor returns a logic low.
        Serial.println("Rain: Yes");
      }
      else { // Print no if rain sensor returns a logic high.
        Serial.println("Rain: No");
      }

      // Create the data string for writing into a .csv file.
      dataStr = String(now.second()) + ',' + String(now.minute()) + ',' + String(now.hour()) + ',' +
        String(now.day()) + ',' + String(now.year()) + ',' + String(data.temperature) + ',' + String(data.humidity)
        + ',' + String(data.pressure) + ',' + String(data.altitude) + ',' + String(data.rpm) + ',' +
        String(data.indexVOC) + ',' + String(data.indexUV) + ',' + String(data.rain);
      dataFile.println(dataStr); // Write the data into the file.
      dataFile.close(); // Close the file.
      hasReceived = false; // Reset receive status.
    }
  }

  delay(200);
}

void receiveEvent() { // Read the data on the I2C bus during a receive event.
  Wire.readBytes((byte *)&data, sizeof(dataPacket));
  hasReceived = true; // Update the receive status.
}