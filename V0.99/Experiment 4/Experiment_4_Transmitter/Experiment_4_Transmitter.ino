/*
Title: Weather Station Transmittter Experiment 4
Programmer: Justin Lam
Date: 15/02/23
Description: Experiment to test RTC and transmitting temperature, humidity, pressure, wind speed, VOC index and if it is raining.
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <dht.h>
#include <DFRobot_BMP3XX.h>
#include <DFRobot_SGP40.h>
#include <RTClib.h>

#define CALIBRATE_ABSOLUTE_DIFFERENCE
#define NUM_PULSES 12 // Number of pulses per rotation

const int CEPin = 7; // nRF24 chip enable pin.
const int CSNPin = 8; // nRF24 chip select pin.
const int DHTPin = 24; // DHT22 pin.
const int rainSensorPin = 25; // Rain sensor pin.
const int powerPin = 26; // Rain sensor power pin.
const int ledPin = 27; // Transmit debug LED pin.
const int channelAPin = 2; // EC12E channel A (CLK) pin.
const int channelBPin = 3; // EC12E channel B (DT) pin.
const float rpmToRad = 0.10471975512;
const float radToDeg = 57.29578;
const byte address[6] = "00001"; // Define the address through which the radios will use to communicate.
int readDHT = 0;
int resultBMP388 = 0; // Integer to store initialisation status of the BMP388.
int curSecond = 0; // Integer to store the current runtime.
int prevSecond = 0; // Integer to store the previous runtime.
volatile long pulseCount = 0; // Number of pulses.
float tempDHT = 0.0;
float tempBMP = 0.0;
float altitude = 0.0;
float samplingPeriod = 0.0;
float samplingFrequency = 0.0;
float angularVelocityRad = 0.0; // Velocity in radians per second.
float angularVelocityDeg = 0.0; // Velocity in degrees per second.
char days[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String timeDate = "";
bool isStart = false; // Boolean value to tell if the transmitter as booted within the previous second.
bool direction = false; // Rotational direction.

struct dataPackage { // Declaration and definition of a struct that contains the data.
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  float rpm = 0.0;
  int index = 0;
  int rain = 1;
};

RF24 radio(CEPin, CSNPin); // Declare and initialise the radio where CE and CSN are digital pins 7 and 8 respectively.
dht DHT; // Declare and initialise DHT22 object.
DFRobot_BMP388_I2C BMP388(&Wire, BMP388.eSDOVDD); // Declare and initialise the BMP388 setting I2C address to 0x77.
DFRobot_SGP40 SGP40(&Wire); // Declare and initialise the SGP40 setting I2C address to 0x59.
RTC_DS3231 RTC; // Declare and initialise the DS3231 setting I2C address to 0x68.
dataPackage data; // Declare the data struct named "data".
DateTime now; // Declare and initialise data object.

void setup() {
  Serial.begin(9600); // Initialise serial communication.
  pinMode(ledPin, OUTPUT); // Set the debug LED to digital pin 27 and define it as an output.

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
    Serial.println("nRF24 Initialised!");
  }

  radio.openWritingPipe(address); // Set the address for transmission.
  radio.setPALevel(RF24_PA_LOW); // Set the amplification of the radio to low.
  radio.stopListening(); // Start transmitting.

  // DS3231 Initialisation
  while (!RTC.begin()) { // Check if the module is connected.
    Serial.println("Error: DS3231 not detected!");
    Serial.flush();
    delay(1000);
  }

  if (!RTC.lostPower()) { // Check if the RTC has lost power and reset the time.
    Serial.println("RTC has lost power, resetting the time!");
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // BMP388 Initialisation
  while (ERR_OK != (resultBMP388 = BMP388.begin())) { // Start BMP388 operation.
    if (ERR_DATA_BUS == resultBMP388) { // Check if the BMP388 is connected to I2C.
      Serial.println("BMP388 data bus error!");
    }
    else if (ERR_IC_VERSION == resultBMP388) { // Check if the BMP388 chip is the correct version.
      Serial.println("BMP388 chip versions do not match!");
    }

    delay(3000);
  }

  while (!BMP388.setSamplingMode(BMP388.eUltraPrecision)) { // Set the sampling mode of the BMP388.
    Serial.println("Error: Set sampling mode failed, retrying...");
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
  Serial.println("BMP388 Initialised!");

  // SGP40 Initialisation
  Serial.println("SGP40 preheating, please wait 10 seconds...");

  while (!SGP40.begin(10000)) {
    Serial.println("Error: Failed to intialise the SGP40!"); // Start operation of the sensor, needs at least 10s to heat the chip.
    delay(1000);
  }

  Serial.println("SGP40 Initialised!");

  // EC12E Intialisation
  pinMode(channelAPin, INPUT_PULLUP); // Set channel A to digital pin 2 with a pull-up resistor.
  pinMode(channelBPin, INPUT_PULLUP); // Set channel B to digital pin 3 with a pull-up resistor.
  attachInterrupt(digitalPinToInterrupt(channelAPin), pulseRead, RISING); // Setup an innterupt to read everytime a rising edge pulse is detected.
  isStart = true; // The transmitter has just booted up.

  // Rain Sensor Intitalisation
  pinMode(rainSensorPin, INPUT); // Set the rain sensor output to digital pin 25.
  pinMode(powerPin, OUTPUT); // Set the power pin of the rain sensor to digital pin 26.
}

void loop() {
  now = RTC.now();
  curSecond = now.second(); // Get the current runtime second.

  if (curSecond != prevSecond && !isStart) { // Check to see if one second has passed and the transmitter has not just booted.
    prevSecond = curSecond; // Save the current second.
    getWindSpeed(); // Measure the wind speed.
  }
  
  if ((now.second() % 5) == 0 && !isStart) { // Start measuring and transmitting data every 5 seconds if the transmitter has not just booted.
    readDHT = DHT.read22(DHTPin); // Read the DHT22 from digital pin 24.
    tempDHT = DHT.temperature; // Get the temperature from the DHT22.
    tempBMP = BMP388.readTempC(); // Get the temperature from the BMP388.
    digitalWrite(powerPin, HIGH); // Turn on comparator module and rain sensor.
    delay(500);
    data.rain = digitalRead(rainSensorPin); // Read the status of the rain sensor.
    digitalWrite(powerPin, LOW); // Turn off the comparator module and rain sensor.
    data.temperature = (tempDHT + tempBMP) / 2; // Take the average temperature.
    data.humidity = DHT.humidity; // Get the humidity.
    data.pressure = BMP388.readPressPa(); // Get the atmospheric pressure.
    altitude = BMP388.readAltitudeM(); // Get the altitude.
    data.index = SGP40.getVoclndex(); // Get the VOC index.
    radio.write(&data, sizeof(dataPackage)); // Transmit the data.
    digitalWrite(ledPin, HIGH); // Turn on the debug LED for every transmission.
    // Print the time and date data was transmitted on the serial monitor.
    Serial.print("Transmitted at: ");
    timeDate = String(now.hour()) + ':' + String(now.minute()) + ':' + String(now.second()) + ' ' + String(now.day())
      + '/' + String(now.month()) + '/' + String(now.year()) + ' ' + days[now.dayOfTheWeek()];
    Serial.println(timeDate);
    // Print all the data on the serial monitor.
    Serial.print(" Temperature: ");
    Serial.print(data.temperature);
    Serial.print("C | Humidity: ");
    Serial.print(data.humidity);
    Serial.print("% | Pressure: ");
    Serial.print(data.pressure);
    Serial.print("Pa | Altitude: ");
    Serial.print(altitude);
    Serial.println("m");
    Serial.print("VOC Index: ");
    Serial.print(data.index);
    Serial.print(" | Wind Speed: ");
    Serial.print(data.rpm);
    Serial.print(" RPM (");
    Serial.print(angularVelocityDeg);
    Serial.print("Deg/s) | ");

    if (data.rain == 0) { // Print yes of rain sensor returns a logic low.
      Serial.println("Rain: Yes");
    }
    else { // Print no if rain sensor returns a logic high.
      Serial.println("Rain: No");
    }

    delay(500);
    digitalWrite(ledPin, LOW); // Turn off the debug LED at the end of each transmission cycle.
  }
  else if (isStart) { // If the transmitter just booted, wait for one second.
    delay(1000);
    isStart = false; // After one second the transmitter has not just booted up.
    pulseCount = 0; // Reset the number of pulses.
  }
}

void pulseRead() { // Count the number of pulses and determine direction of rotation.
  int readVal = digitalRead(channelBPin); // Read the value coming from channel B.

  if (readVal == 0) { // If the the value is logic low, then rotation is counter clockwise.
    direction = false;
  }
  else { // Else, rotation is clockwise.
    direction = true;
  }

  if (!direction) { // Increase number of pulses if in the counter clockwise direction.
    pulseCount++;
  }
  else { // Decrease number of pulses if in the clockwise direction.
    pulseCount--;
  }
}

void getWindSpeed() { // Calculate the windspeed.
  data.rpm = (float)((pulseCount * 60) / NUM_PULSES); // Calculate the rpm.
  angularVelocityRad = data.rpm * rpmToRad; // Calculate the angular velocity in Rad/s.
  angularVelocityDeg = angularVelocityRad * radToDeg; // Calculate the angular velocity in Deg/s.
  pulseCount = 0; // Reset the pulse count.
}
