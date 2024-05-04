/*
 * Programmer: Justin Lam
 * Date: 03/05/24
 * Description: Transmit humidity, temprature, pressure, altitude, air quality, 
 *              and UV index data via Lo-Ra radio every 10s.
*/

#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <Adafruit_SHTC3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <DFRobot_SGP40.h>
#include <RTClib.h>
/*
 *--------------------------------------------*
 * Add any necessary library inclusions here. *
 *--------------------------------------------*
*/

#define RFM95_IRQ 3 // RFM95 interrupt pin.
#define RFM95_RST 4 // RFM95 reset pin.
#define RFM95_CS 8 // RFM95 slave select pin.
#define RFM95_FREQ 433.0 // Radio frequency of 433Hz.
#define SEA_LEVEL_PRESSURE_HPA (1013.25) // Atmospheric pressure at sea level.
/*
 *---------------------------------------------------------------------------------*
 * Add preprocessor definitions here.                                              *
 * All definitions must be named in all caps separated by an underscore character. *
 * Example: "#define MY_DEFINITION 0".                                             *
 *---------------------------------------------------------------------------------*
*/

float calcUVIndex(); // Calculate the UV index according to the value coming from the sensor.
/*
 *------------------------------------------------------------------------*
 * Add any function forward declarations here.                            *
 * Function names should be in camel case: "void myFunction();".          *
 * The order of arguments are as follows: int->float->char->string->bool. *
 *------------------------------------------------------------------------*
 */

const int UVSensorPin = A1;
uint8_t buffLen = 0;
uint8_t dataLen = 0;
uint8_t buffer[RH_RF95_MAX_MESSAGE_LEN] = { 0 };
uint8_t data[RH_RF95_MAX_MESSAGE_LEN] = { 0 };
String timeDate = "";
sensors_event_t humi, temp;
DateTime now;
/*
 *-------------------------------------------------------------------------*
 * Add any variables that are need to store sensor data here.              *
 * The order of variables are as follows: int->float->char->string->bool.  *
 * Always decalre and initialise your variables with a default value.      *
 * You do not need to intialise any custom datatype with a default value.  *
 * Use camel case for variable names: "int myVariable = 0;".               *
 *-------------------------------------------------------------------------*
*/

RH_RF95 rfm95(RFM95_CS, RFM95_IRQ); // Declare RFM95 radio object.
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3(); // Declare SHTC3 sensor object.
Adafruit_BMP3XX bmp388; // Declare BMP388 sensor object.
DFRobot_SGP40 sgp40(&Wire); // Declare SGP40 sensor object.
RTC_DS3231 rtc;
/*
 *---------------------------------------------*
 * Add any object decalrations here.           *
 * Name of object should be in all lower case. *
 *---------------------------------------------*
*/

void setup() {
  Serial.begin(115200); // Intialise serial communication.
  pinMode(RFM95_RST, OUTPUT); // Set RFM95 reset pin to output.
  // Reset the RFM95 radio.
  digitalWrite(RFM95_RST, HIGH);
  delay(20);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialise the RFM95 radio.
  while (!rfm95.init()) {
    Serial.println("Error: Failed to initialise RFM95!");
    Serial.println("Retrying...");
    delay(1000);
  }

  // Set the frequency of the RFM95 radio.
  while (!rfm95.setFrequency(RFM95_FREQ)) {
    Serial.println("Error: Failed to set frequency to to 433HZ!");
    Serial.println("Retrying...");
    delay(1000);
  }

  // Print the frequency to serial monitor.
  Serial.print("Frequency set to ");
  Serial.print(RFM95_FREQ);
  Serial.println("Hz.");
  rfm95.setTxPower(23, false); // Set the transmission power level.
  Serial.println("RFM95 successfully intitialised!");

  // Initialise the SHTC3.
  while (!shtc3.begin()) {
    Serial.println("Error: Failed to initialise SHTC3!");
    Serial.println("Retrying...");
    delay(1000);
  }

  Serial.println("SHTC successfully initialised!");

  // Initialise the BMP388.
  while (!bmp388.begin_I2C()) {
    Serial.println("Error: Failed to initialise BMP388!");
    Serial.println("Retrying...");
    delay(1000);
  }

  bmp388.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp388.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp388.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp388.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("BMP388 successfully initialised!");

  // Initialise the SGP40.
  Serial.println("SGP40 preheating, please wait 10 seconds...");

  while (!sgp40.begin(10000)) {
    Serial.println("Error: Failed to initialise SGP40!");
    Serial.println("Retrying...");
    delay(1000);
  }

  Serial.println("SGP40 successfully initialised!");

  // Initialise the DS3231.
  while (!rtc.begin()) {
    Serial.println("Error: Faile to initialise DS3231!");
    Serial.println("Retrying...");
    delay(1000);
  }

  if (rtc.lostPower()) { // Reset the time and date if power loss occured.
    Serial.println("RTC has lost power, resetting the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial.println("DS3231 successfully initialised!");
  /*
   *------------------------------------------------------------------------*
   * Add the initialisation of any modules here.                            *
   * Use a while loop to check for the intiialisation status as done above. *
   * Use a delay of 1s when retrying initialisation.                        *
   *------------------------------------------------------------------------*
  */

  delay(500);

}

void loop() {
  now = rtc.now();

  if (now.second() % 10 == 0) // Transmit data every 10 seconds.
  {
    shtc3.getEvent(&humi, &temp);

    if (!bmp388.performReading()) {
      Serial.println("Error: Failed to perform BMP388 reading!");
    }
    /*
    *--------------------------*
    * Add sensor reading here. *
    *--------------------------*
    */
    // Format string for time and date.
    timeDate = String(now.hour()) + ':' + String(now.minute()) + ':' + String(now.second()) + ' ' + String(now.day()) + '/' + String(now.month()) + '/' + String(now.year());
    // Format a string that contains the data separated by spaces.
    dataLen = sprintf((char *)data, "%f %f %f %f %f %f", humi.relative_humidity, temp.temperature, (float)(bmp388.pressure / 100.0), (float)bmp388.readAltitude(SEA_LEVEL_PRESSURE_HPA), (float)sgp40.getVoclndex(), calcUVIndex());
    /*
    *-----------------------------------------------------------------------------------------------------------------------------------------------*
    * The data is stored in an array of unsigned 8-bit integers, where the individual characters are stored in ASCII values.                        *
    * When formatting the string, the array of integers is type casted into an array of characters.                                                 *
    * To add data to the data string just add the neccessay format specifier in the 2nd argument followed by the variable name in the 3rd argument. *
    *-----------------------------------------------------------------------------------------------------------------------------------------------*
    */
    Serial.println(timeDate); // Print the time and the date to serial monitor.
    Serial.println((char *)data); // Print data to serial monitor.
    rfm95.send(data, dataLen); // Send transmit the data.
    delay(20);
    rfm95.waitPacketSent(); // Wait until the data has been transmitted.
    buffLen = sizeof(buffer);

    // Receive the acknowledgement packet.
    if (rfm95.waitAvailableTimeout(1000)) {
      if (rfm95.recv(buffer, &buffLen)) {
        Serial.print("Acknowledge: ");
        Serial.println((char *)buffer); // Print out the acknowledgement packet on the serial monitor.
      }
      else { // Print failure message if acknowledgement packet has failed.
        Serial.println("Failed to confirm acknowledgement!");
      }
    }
    else {
      Serial.println("No acknowledgement received!");
    }

    delay(700); // Delay to wait for the RTC to count to the next second.
  }
}

// Calculate the UV index according to the value coming from the sensor.
float calcUVIndex() {
  int readVal = 0;
  float UVIndex = 0.0;

  // Calculate the UV index based on the anlogue values.
  if (readVal <= 10) { // UV Index 0.
    UVIndex = 0.0;
  }
  else if (readVal <= 25.782) {
    UVIndex = (0.02778 * readVal) - 0.2778;
    UVIndex = (float)round(UVIndex);
  }
  else {
    UVIndex = (0.05147 * readVal) - 1.327;
    UVIndex = (float)round(UVIndex);
  }

  return UVIndex;
}
/*
 *-----------------------------------------------*
 * All function definitions are to be made here. *
 *-----------------------------------------------*
*/