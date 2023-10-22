/*
Title: Remote Weather Station Transmittter V0.99
Programmer: Justin Lam
Date: 20/02/23
Description: Transmitter for the remote weather station, reads data from the sensors and transmits the data at regular time intervals or when a data request is received.
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <dht.h>
#include <DFRobot_BMP3XX.h>
#include <DFRobot_SGP40.h>
#include <RTClib.h>

#define CALIBRATE_ABSOLUTE_DIFFERENCE
#define NUM_PULSES 12 // Number of pulses per rotation.

const int UVSensorPin = A0; // UV sensor pin.
const int channelAPin = 2; // EC12E channel A (CLK) pin.
const int channelBPin = 3; // EC12E channel B (DT) pin.
const int CEPin = 7; // nRF24 chip enable pin.
const int CSNPin = 8; // nRF24 chip select pin.
const int DHTPin = 24; // DHT22 pin.
const int rainSensorPin = 25; // Rain sensor pin.
const int powerPin = 26; // Rain sensor power pin.
const int transmitLedPin = 27; // Transmit debug LED pin.
const int failureLedPin = 28; // Failure to transmit debug LED pin.
const int retryLedPin = 29; // Retrying to transmit debug LED pin;
const float rpmToRad = 0.10471975512;
const float radToDeg = 57.29578;
const byte address[2][6] = {"00001", "00002"}; // Define the addresses through which the radios will use to communicate.
int readDHT = 0;
int readUV = 0;
int resultBMP388 = 0; // Integer to store initialisation status of the BMP388.
int UVIndex = 0;
int curSecond = 0; // Integer to store the current runtime.
int prevSecond = 0; // Integer to store the previous runtime.
int numRetry = 0;
volatile long pulseCount = 0; // Number of pulses.
float tempDHT = 0.0;
float tempBMP = 0.0;
float samplingPeriod = 0.0;
float samplingFrequency = 0.0;
float angularVelocityRad = 0.0; // Velocity in radians per second.
float angularVelocityDeg = 0.0; // Velocity in degrees per second.
char days[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String timeDate = "";
bool isStart = false; // Boolean value to tell if the transmitter has booted within the previous second.
bool direction = false; // Rotational direction.
bool isSuccess = false; // Boolean value to tell if transmission was a success.
bool ackSuccess = false; // Boolean value to tell if acknowledgement was a success.
bool startTransmit = false; // Boolean value to tell if to transmit the data request.
 
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

struct buttonPacket { // Declaration and definition of a struct that contains the button state for transmission request.
  bool buttonState = false;
};

struct ackPacket { // Declaration and definition of a struct that contains the acknowledge data.
  bool isAck = false;
};

RF24 radio(CEPin, CSNPin); // Declare and initialise the radio where CE and CSN are digital pins 7 and 8 respectively.
dht DHT; // Declare and initialise DHT22 object.
DFRobot_BMP388_I2C BMP388(&Wire, BMP388.eSDOVDD); // Declare and initialise the BMP388 setting I2C address to 0x77.
DFRobot_SGP40 SGP40(&Wire); // Declare and initialise the SGP40 setting I2C address to 0x59.
RTC_DS3231 RTC; // Declare and initialise the DS3231 setting I2C address to 0x68.
dataPacket data; // Declare the data packet named "data".
DateTime now;
buttonPacket button; // Declare the button packet named "button".
ackPacket received; // Declare the acknowledge packet named "received".

void setup() {
  Serial.begin(9600); // Initialise serial communication.

  /* LED Initialisation */
  // Assign the LED pins as outputs.
  pinMode(transmitLedPin, OUTPUT);
  pinMode(failureLedPin, OUTPUT);
  pinMode(retryLedPin, OUTPUT);
  // Turn off all the LEDs.
  digitalWrite(transmitLedPin, LOW);
  digitalWrite(failureLedPin, LOW);
  digitalWrite(retryLedPin, LOW);

  // Not required
  /*
  #ifndef ESP8266
  while (!Serial) { // Check if serial communciation has been established.
    digitalWrite(failureLedPin, HIGH);
    delay(500);
    digitalWrite(failureLedPin, LOW);
    delay(500);
  }
  #endif
  */

  /* Radio Initialisation */
  if (!radio.begin()) { // Check if the radio module is responding.
    Serial.println("Error: nRF24 radio not responding!");
    Serial.println("Please check wiring...");
    digitalWrite(failureLedPin, HIGH);
    while(1);
  }
  else {
    radio.openWritingPipe(address[0]); // Set the address for transmission.
    radio.openReadingPipe(1, address[1]); // Set the address for reception.
    //radio.writeAckPayload(1, &received, sizeof(ackPacket));
    radio.setPALevel(RF24_PA_LOW); // Set the power amplification of the radio to low.
    radio.enableAckPayload(); // Enable the acknowledgement payload.
    Serial.println("nRF24 Initialised!");
  }

  /* DS3231 Initialisation */
  while (!RTC.begin()) { // Check if the clock module is responding.
    Serial.println("Error: DS3231 not detected!");
    Serial.flush();
    digitalWrite(failureLedPin, HIGH);
    delay(500);
    digitalWrite(failureLedPin, LOW);
    delay(500);
  }

  if (RTC.lostPower()) { // Check if the clock module has lost power and reset the time.
    Serial.println("RTC has lost power, resetting the time!");
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial.println("DS3231 Initialised!");

  /* BMP388 Initialisation */
  while (ERR_OK != (resultBMP388 = BMP388.begin())) { // Check if the BMP388 is responding.
    if (ERR_DATA_BUS == resultBMP388) { // Check if the BMP388 is connected to I2C.
      Serial.println("BMP388 data bus error!");
    }
    else if (ERR_IC_VERSION == resultBMP388) { // Check if the BMP388 chip is the correct version.
      Serial.println("BMP388 chip versions do not match!");
    }

    digitalWrite(failureLedPin, HIGH);
    delay(500);
    digitalWrite(failureLedPin, LOW);
    delay(500);
  }

  #ifdef CALIBRATE_ABSOLUTE_DIFFERENCE
  if (BMP388.calibratedAbsoluteDifference(56.0)) { // Calibrate the altitude (University of Liverpool: 56m).
    Serial.println("Absolute difference base value set successfully!");
  }
  #endif

  while (!BMP388.setSamplingMode(BMP388.eUltraPrecision)) { // Set the sampling mode of the BMP388.
    Serial.println("Error: Set sampling mode failed, retrying...");
    digitalWrite(failureLedPin, HIGH);
    delay(500);
    digitalWrite(failureLedPin, LOW);
    delay(500);
  }

  samplingPeriod = BMP388.getSamplingPeriodUS(); // Get the sampling period of the BMP388
  Serial.print("Sampling Period: ");
  Serial.print(samplingPeriod);
  Serial.println("us");
  samplingFrequency = 1000000 / samplingPeriod; // Calculate the sampling frequency of the BMP388.
  Serial.print("Sampling Frequency: ");
  Serial.print(samplingFrequency);
  Serial.println("Hz");
  Serial.println("BMP388 Initialised!");

  /* SGP40 Initialisation */
  Serial.println("SGP40 preheating, please wait 10 seconds...");

  while (!SGP40.begin(10000)) { // Start operation of the sensor, needs at least 10 seconds to heat the chip.
    Serial.println("Error: Failed to intialise the SGP40!");
    digitalWrite(failureLedPin, HIGH);
    delay(500);
    digitalWrite(failureLedPin, LOW);
    delay(500);
  }

  Serial.println("SGP40 Initialised!");

  /* EC12E Initialisation */
  pinMode(channelAPin, INPUT_PULLUP); // Set channel A to digital pin 2 with a pull-up resistor.
  pinMode(channelBPin, INPUT_PULLUP); // Set channel B to digital pin 3 with a pull-up resistor.
  attachInterrupt(digitalPinToInterrupt(channelAPin), pulseRead, RISING); // Setup an interrupt to execute function everytime a rising edge pulse is detected.
  //isStart = true;

  /* Rain Sensor Initialisation */
  pinMode(rainSensorPin, INPUT); // Set the rain sensor output to digital pin 25.
  pinMode(powerPin, OUTPUT); // Set the power pin of the rain sensor to digital pin 26.
  Serial.println("All components and sensors initialised!");
  isStart = true; // The transmitter has just booted up.
}

void loop() {
  now = RTC.now(); // Get the current time and date.
  curSecond = now.second(); // Get the current second.

  if (curSecond != prevSecond && !isStart) { // Measure windspeed if at least one second has passed and if the transmitter has not just booted.
    prevSecond = curSecond; // Save the current second.
    getWindSpeed();
  }

  if (((now.second() % 10 == 0) || startTransmit) && !isStart) { // Start measuring and transmitting data every 10 seconds if the transmitter has not just booted.
    readDHT = DHT.read22(DHTPin); // Read the DHT22 from digital pin 24.
    tempDHT = DHT.temperature; // Get the temperature from the DHT22.
    tempBMP = BMP388.readTempC(); // Get the temperature from the BMP388.
    digitalWrite(powerPin, HIGH); // Turn on comparator module and rain sensor.
    delay(500); // Wait for the circuit transient to stabilise.
    data.rain = digitalRead(rainSensorPin); // Read the status of the rain sensor.
    digitalWrite(powerPin, LOW); // Turn off the comparator module and rain sensor.
    data.temperature = (tempDHT + tempBMP) / 2; // Take the average temperature.
    data.humidity = DHT.humidity; // Get the humidity.
    data.pressure = BMP388.readPressPa() + 1150; // Get the atmospheric pressure and calibrate.
    data.altitude = BMP388.readAltitudeM(); // Get the altitude.
    data.indexVOC = SGP40.getVoclndex(); // Get the VOC index.    
    data.indexUV = getUVIndex(); // Get the UV index.
    digitalWrite(transmitLedPin, HIGH); // Turn on the debug LED for every transmission.
    radio.stopListening(); // Enable transmitting.
    isSuccess = radio.write(&data, sizeof(dataPacket)); // Transmit the data packet.
    delay(250);
    numRetry = 0; // Reset the number of retries.
    checkTransmission(isSuccess, numRetry); // Check if data transmission was a success.

    if (isSuccess) { // Check if the data transmission was a success.
    // Print the time, date and data on the serial monitor.
      Serial.print("Transmitted at: ");
      timeDate = String(now.hour()) + ':' + String(now.minute()) + ':' + String(now.second()) + ' ' + String(now.day())
        + '/' + String(now.month()) + '/' + String(now.year()) + ' ' + days[now.dayOfTheWeek()];
      Serial.println(timeDate);
      Serial.print(" Temperature: ");
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
       Serial.print(" RPM (");
       Serial.print(angularVelocityDeg);
       Serial.print("Deg/s) | ");

       if (data.rain == 0) { // Print yes of rain sensor returns a logic low.
         Serial.println("Rain: Yes");
       }
       else { // Print no if rain sensor returns a logic high.
         Serial.println("Rain: No");
       }
    }

    delay(250);
    digitalWrite(transmitLedPin, LOW); // Turn off the debug LED at the end of each transmission cycle.
    digitalWrite(failureLedPin, LOW);
    startTransmit = false; // Reset the transmission status.
  }
  else if (isStart) { // If the transmitter just booted, wait for one second.
    delay(1000);
    isStart = false; // After one second the transmitter has not just booted up.
    pulseCount = 0; // Reset the number of pulses.
  }
  else { // On standby start listening for data requests.
    radio.startListening(); // Enable receiving.

    if (radio.available()) { // Check if a data request has been sent.
      radio.read(&button, sizeof(buttonPacket)); // Read the data request packet.
      received.isAck = true; // The transmitter has received the data request packet.
      ackSuccess = radio.writeAckPayload(1, &received, sizeof(ackPacket)); // Send the acknowledgement to the receiver master.

      if (ackSuccess) { // If the acknowledgement packet was successfully sent, set the transmitter to start transmitting data on the next loop.
        startTransmit = button.buttonState; 
        Serial.println("Request from receiver to transmit data!");
      }
      else {
        Serial.println("Failed to acknowledge request to transmit data");
      }
    }    
  }
}

void pulseRead() { // Count the number of pulses and determine direction of rotation.
  int readVal = digitalRead(channelBPin); // Read the value coming from channel B.

  if (readVal == 0) { // If the the value is logic low, then rotation is counter clockwise.
    direction = false;
  }
  else { // Otherwise, rotation is clockwise.
    direction = true;
  }

  if (!direction) {// Increase number of pulses if in the counter clockwise direction.
    pulseCount++;
  }
  else { // Decrease number of pulses if in the clockwise direction.
    pulseCount--;
  }
}

void getWindSpeed() { // Calculate the windspeed in revolution per minute (RPM).
  data.rpm = (float)((pulseCount * 60) / NUM_PULSES); // Calculate the RPM.
  angularVelocityRad = data.rpm * rpmToRad; // Calculate the angular velocity in Rad/s.
  angularVelocityDeg = angularVelocityRad * radToDeg; // Calculate the angular velocity in Deg/s.
  pulseCount = 0; // Reset the pulse count.
}

void checkTransmission(bool status, int retry) { // Recursively check if the data transmission was successful, if not retry transmission.
  digitalWrite(retryLedPin, LOW); // Reset the retry LED to off.
  delay(250);

  if (status) { // Check if the data packet was sent.
    if (radio.isAckPayloadAvailable()) { // Check if the receiver master has sent an acknowledgement packet.
      radio.read(&received, sizeof(ackPacket)); // Read the acknowledgement packet.

      if (received.isAck) { // Confirm acknowledgement of data reception.
        status = true;
        Serial.println("Data was receieved by the receiver!");
      }
      else {
        status = false;
        Serial.println("Data was not received by the receiver!");
      }
    }
  }
  else if (!status && retry < 5) { // If the data packet was not received, then resend the data packet, for a maximum number of 5 retries.
    digitalWrite(failureLedPin, HIGH);
    Serial.println("Transmission failed, retrying...");
    Serial.println(retry);
    digitalWrite(retryLedPin, HIGH);
    status = radio.write(&data, sizeof(dataPacket)); // Resend the data packet.
    digitalWrite(retryLedPin, LOW);
    checkTransmission(status, retry + 1); // Recursively check if the retry was successful.
  }
  else if (retry == 5) { // If the maximum of retries has been reached, move on.
    Serial.println("Error: Transmission failed, check receiver!");
  }  
}

int getUVIndex() { // Read the UV sensor and calculate and return the UV index.
  readUV = analogRead(UVSensorPin);

  if (readUV < 23) {
    UVIndex = 0;
  }
  else if (readUV > 23 && readUV < 46) {
    UVIndex = 1;
  }
  else if (readUV > 46 && readUV < 65) {
    UVIndex = 2;
  }
  else if (readUV > 65 && readUV < 83) {
    UVIndex = 3;
  }
  else if (readUV > 83 && readUV < 103) {
    UVIndex = 4;
  }
  else if (readUV > 103 && readUV < 124) {
    UVIndex = 5;
  }
  else if (readUV > 124 && readUV < 142) {
    UVIndex = 6;
  }
  else if (readUV > 142 && readUV < 162) {
    UVIndex = 7;
  }
  else if (readUV > 162 && readUV < 180) {
    UVIndex = 8;
  }
  else if (readUV > 180 && readUV < 200) {
    UVIndex = 9;
  }
  else if (readUV > 200 && readUV < 221) {
    UVIndex = 10;
  }
  else if (readUV > 221 && readUV < 240) {
    UVIndex = 11;
  }
  else if (readUV > 240) {
    UVIndex = 12;
  }

  return UVIndex;
}