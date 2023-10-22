/*
Title: Weather Station Receiver Experiment 7
Programmer: Justin Lam
Date: 26/02/23
Description: Data logging experiment, slave arduino that receives the data and sends in via I2C.
*/

#include <Wire.h>
#include <RTClib.h>
#include <SD.h>

const int CSPin = 10; // SD card chip select.
char days[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
bool hasReceived = false;
String timeDate = ""; // String to store time and date.
String dataStr = ""; // String to store data for writing.

struct dataPackage { // Declaration and definition of a struct that contains the data.
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  float altitude = 0.0;
  float rpm = 0.0;
  int index = 0;
  int rain = 1;
};

RTC_DS3231 clock; // Declare and initialise the DS3231 setting I2C address to 0x68.
dataPackage data; // Declare and initialise data object.
DateTime now; // Declare and initialise data object.
File dataFile; // File object for writing data.

void setup() {
  Serial.begin(9600); // Begin serial communciation.
  Wire.begin(6); // Begin I2C communication on adrress 6.

  #ifndef ESP8266
  while (!Serial) { // Check if the serial port has connected.
    Serial.println("Error: failed to start serial communication!");
    delay(2000);
  }
  #endif

  // SD Card Initialisation
  Serial.println("Intitalising SD card...");

  if (!SD.begin(CSPin)) { // Check to see if SD card has initialised.
    Serial.println("Error: SD card not responding!");
    while(1);
  }
  else {
    Serial.println("SD Card Initialised!");
  }

  // DS3231 Initialisation
  while (!clock.begin()) { // Check if the module is connected.
    Serial.println("Error: DS3231 not detected!");
    Serial.flush();
    delay(2000);
  }

  if (clock.lostPower()) { // Check if the RTC has lost power and reset the time.
    Serial.println("RTC has lost power, resetting the time!");
    clock.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial.println("DS3231 Initialised!");
  Wire.onReceive(receiveEvent); // Register a receive event.
}

void loop() {
  if (hasReceived) { // Check if data was received on the I2C bus from master arduino.
    now = clock.now(); // Get the current time and date.
    dataFile = SD.open("WEATHE~1.CSV", FILE_WRITE); // Open file in SD card for writing.

    if (dataFile) { // Check if file is open.
      // Make a time and date string and print on the serial monitor.       
      Serial.println("Logging Data...");
      timeDate = String(now.hour()) + ':' + String(now.minute()) + ':' + String(now.second()) + ' ' + String(now.day())
        + '/' + String(now.month()) + '/' + String(now.year()) + ' ' + days[now.dayOfTheWeek()];
      Serial.print("Received at: ");
      Serial.println(timeDate);
      // Print the received data on the serial monitor.
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

      // Create the data string for writing into a .csv file.
      dataStr = String(now.second()) + ',' + String(now.minute()) + ',' + String(now.hour()) + ',' +
        String(now.day()) + ',' + String(now.year()) + ',' + String(data.temperature) + ',' + String(data.humidity)
        + ',' + String(data.pressure) + ',' + String(data.altitude) + ',' + String(data.rpm) + ',' + 
        String(data.index) + ',' + String(data.rain);
      dataFile.println(dataStr);
      dataFile.close(); // Close the file.
      hasReceived = false; // Reset receive status.
    }
  }

  delay(250);
}

void receiveEvent() { // Read the data on the I2C bus.
  Wire.readBytes((byte *)&data, sizeof(dataPackage));
  hasReceived = true; // Update the receive status.
}