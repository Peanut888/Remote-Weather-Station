/*
Title: Weather Station Transmittter Experiment 3
Programmer: Justin Lam
Date: 13/02/23
Description: Experiment to test RTC and transmitting temperature, humidity, pressure, altitude, VOC index and if it is raining.
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <dht.h>
#include <DFRobot_BMP3XX.h>
#include <DFRobot_SGP40.h>
#include <RTClib.h>

#define CE_PIN 7 // Define chip enable pin.
#define CSN_PIN 8 // Define chip select pin.
#define CALIBRATE_ABSOLUTE_DIFFERENCE

const int DHTPin = 24; // Store the digital pin number for the sensor.
const int rainSensorPin = 25;
const int sensorPowerPin = 26;
const byte address[6] = "00001"; // Define the address through which the radios will use to communicate.
int readDHT = 0;
int resultBMP388 = 0; // Integer to store initialisation status of the BMP388.
int second = 0;
int minute = 0;
int hour = 0;
int date = 0;
int month = 0;
int year = 0;
float tempDHT = 0.0;
float tempBMP = 0.0;
float samplingPeriod = 0.0;
float samplingFrequency = 0.0;
char days[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String timeDate = ""; // String to store the time and date.
String day = "";

struct dataPackage { // Declaration and definition of a struct that contains the data.
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  float altitude = 0.0;
  int index = 0;
  int rain = 1;
};

RF24 radio(CE_PIN, CSN_PIN); // Declare and initialise the radio where CE and CSN are digital pins 7 and 8 respectively.
dht DHT; // Declare and initialise sensor object.
DFRobot_BMP388_I2C BMP388(&Wire, BMP388.eSDOVDD); // Declare and initialise the BMP388 setting I2C address to 0x77.
DFRobot_SGP40 SGP40(&Wire); // Declare and initialise the SGP40 setting I2C address to 0x59.
RTC_DS3231 RTC; // Declare and initialise the DS3231 setting I2C address to 0x68.
dataPackage data; // Declare the data struct named "data".
DateTime now; // Declare and initialise data object.

void setup() {
  Serial.begin(9600);
  pinMode(rainSensorPin, INPUT);
  pinMode(sensorPowerPin, OUTPUT);
  digitalWrite(sensorPowerPin, LOW);

  #ifndef ESP8266
  while (!Serial); // Check if the serial port has connected.
  #endif

  // DS3231 Initialisation
  while (!RTC.begin()) { // Check if the module is connected.
    Serial.println("RTC module not present!");
    Serial.flush();
    delay(1000);
  }

  if (RTC.lostPower()) { // Check if the RTC has lost power and reset the time.
    Serial.println("RTC has lost power, resetting the time!");
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // nRF24L01 Initialisation
  radio.begin(); // Start radio operation.
  //radio.setDataRate(RF24_250KBPS); // Set the data rate to 250kb/s.
  //radio.setChannel(120); // Set the channel frequency to channel 120 (2520MHz).
  radio.openWritingPipe(address); // Set the address for transmission.
  radio.setPALevel(RF24_PA_LOW); // Set the amplification of the radio to low.
  radio.stopListening(); // Start transmitting.

  // BMP388 Initialisation
  while (ERR_OK != (resultBMP388 = BMP388.begin())) { // Start BMP388 operation.
    if (ERR_DATA_BUS == resultBMP388) { // Check if the BMP388 is connected to I2C.
      Serial.println("Data bus error!");
    }
    else if (ERR_IC_VERSION == resultBMP388) { // Check if the BMP388 chip is the correct version.
      Serial.println("Chip versions do not match!");
    }

    delay(3000);
  }

  Serial.println("BMP388 begin!");

  while (!BMP388.setSamplingMode(BMP388.eUltraPrecision)) { // Set the sampling mode of the BMP388.
    Serial.println("Set sampling mode failed, retrying...");
    delay(3000);
  }
  
  #ifdef CALIBRATE_ABSOLUTE_DIFFERENCE
  if (BMP388.calibratedAbsoluteDifference(56.0)) { // Calibrate the altitude (University of Liverpool 56m).
    Serial.println("Absolute difference base value set successfully!");
  }
  #endif

  samplingPeriod = BMP388.getSamplingPeriodUS(); // Get the sampling period of the BMP388.
  Serial.print("Sampling Period: ");
  Serial.print(samplingPeriod);
  Serial.println("us");
  samplingFrequency = 1000000 / samplingPeriod; // Calculate the sampling frequency of the BMP388.
  Serial.print("Sampling Frequency: ");
  Serial.print(samplingFrequency);
  Serial.println("Hz");
  Serial.println();
  
  // SGP40 Initialisation
  Serial.println("SGP40 preheating, please wait 10 seconds...");

  while (!SGP40.begin(10000)) { // Start operation of the sensor, needs at least 10s to heat the chip.
    Serial.println("Failed to intialise chip!");
    delay(1000);
  }

  Serial.println("SGP40 initialised!");
  delay(1000);
}

void loop() {
  now = RTC.now(); // Read the time from the RTC.

  if ((now.second() % 10) == 0) { // Read the DHT22 from digital pin 24 of the Arduino Mega.
    readDHT = DHT.read22(DHTPin); // Get the temperature from the DHT22.
    tempDHT = DHT.temperature; // Get the temperature from the DHT22.
    tempBMP = BMP388.readTempC(); // Get the temperature from the BMP388.
    digitalWrite(sensorPowerPin, HIGH); // Turn on comparator module and rain sensor.
    delay(500);
    data.rain = digitalRead(rainSensorPin); // Read the status of the rain sensor.
    digitalWrite(sensorPowerPin, LOW); // Turn off the comparator module and rain sensor.
    data.temperature = (tempDHT + tempBMP) / 2; // Take the average temperature.
    data.humidity = DHT.humidity; // Get the humidity.
    data.pressure = BMP388.readPressPa(); // Get the atmospheric pressure.
    data.altitude = BMP388.readAltitudeM(); // Get the altitude.
    data.index = SGP40.getVoclndex(); // Get the VOC index.
    radio.write(&data, sizeof(dataPackage)); // Transmit the data.
    Serial.print("Transmitted at: ");
    now = RTC.now(); // Read the time from the RTC.
    second = now.second(); // Get the current second.
    minute = now.minute(); // Get the current minute.
    hour = now.hour(); // Get the current hour.
    date = now.day(); // Get the current date.
    month = now.month(); // Get the current month.
    year = now.year(); // Get the current year.
    day = days[now.dayOfTheWeek()]; // Get the current day.
    timeDate = String(second) + ',' + String(minute) + ',' + String(hour) + ',' + String(date) + ','
      + String(month) + ',' + String(year) + ',' + day; // Create a string containing the time and date.
    // Print the time, date and data on the serial monitor.
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
    Serial.print(data.index);

    if (data.rain == 0) {
      Serial.println(" | Rain: Yes"); // Print yes of rain sensor returns a logic low.
    }
    else {
      Serial.println(" | Rain: No"); // Print no if rain sensor returns a logic high.
    }

    delay(500);
  }
  else {
    delay(1000);
  }
}
