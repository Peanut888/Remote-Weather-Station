/*
 * Programmer: Justin Lam, Alhussain Albusaidi
 * Date: 11/05/2024
 * Description: Transmit humidity, temeprature data via Lo-Ra radio.
*/

#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <Adafruit_SHTC3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include "MD5.h"
#include <RTClib.h>
#include <math.h>
#include <DFRobot_SGP40.h>
/*
 *--------------------------------------------*
 * Add any necessary library inclusions here. *
 *--------------------------------------------*
*/


#define SEA_LEVEL_PRESSURE_HPA (1013.25)                                        // * Atmospheric pressure at sea level.
#define RFM95_IRQ 3                                                             // * RFM95 interrupt pin.
#define RFM95_RST 4                                                             // * RFM95 reset pin.
#define RFM95_CS 8                                                              // * RFM95 slave select pin.
#define RFM95_FREQ 433.0                                                        // * Radio frequency of 433Hz.
#define FAILURE_LED_PIN  28                                                     // * Failure to transmit debug LED pin.
#define UVSensorPin A1

#define HASH_SIZE 16                                                            // * 16 octets (8 bits) are required to store 128 bits
#define MAC_ADDRESS "33:b7:ef:28:63:74"                                         // * Example mac
#define PASSKEY "qw@jZTB8vnY!"                                                  // * Example passkey
#define BASE_MAC_ADDR "2b:85:0f:d5:0f:21"                                       // * Example mac, replace with base station mac address
#define BASE_PASSKEY "14M0Itj6Q+C8)p$2"                                         // * Example passkey

#define MESSAGE_SIZE 28                                                         // * Total number of octets in data[]

#define TIME_WINDOW  12                                                         // * Time window in seconds to allow transmission
#define LATENCY_THRESHOLD 5                                                     // * Latency threshold in seconds

#define ackSuccess 0
#define ackFailure 1
#define ackDuplicate 2
#define SUCCESS_INDICATOR 0xFF
#define FAILURE_INDICATOR 0x00
#define DUPLICATE_INDICATOR 0xAA

/*
 *---------------------------------------------------------------------------------*
 * Add preprocessor definitions here.                                              *
 * All definitions must be named in all caps separated by an underscore character. *
 * Example: "#define MY_DEFINITION 0".                                             *
 *---------------------------------------------------------------------------------*
*/
uint8_t* generateHash(const char* data);                                        // * Generate an MD5 hash from the given data.
float calcUVIndex();                                                            // * Calculates the UV index based on the analog value read from the UV sensor.
bool messageStatus(uint32_t unix_time);                                         // * Check if it's time to send a message based on the current UNIX time.
bool acknowledgementPacketVerification(uint8_t* ackMsg);                        // * Verifies the integrity of the acknowledgement packet received from the base station.
void removeExpectedTimestamp(uint32_t timestamp);                               // * Removes the specified timestamp from the array of expected message timestamps.
void sendAcknowledgement();                                                     // * Sends acknowledgement packets to the base station.
void CreateMessage();                                                           // * Constructs the message packet for transmission.

/*
 *------------------------------------------------------------------------*
 * Add any function forward declarations here.                            *
 * Function names should be in camel case: "void myFunction();".          *
 * The order of arguments are as follows: int->float->char->string->bool. *
 *------------------------------------------------------------------------*
 */
int retry_count = 0;
int messages_count = 0;
uint32_t unix_time = 0;
uint32_t id_timestamp = 0;
uint32_t final_unix_time = 0;
uint32_t* expected_messages_timestamps = nullptr;                               // * Pointer to an array storing timestamps of expected messages.
const uint8_t expected[] = {SUCCESS_INDICATOR, SUCCESS_INDICATOR, DUPLICATE_INDICATOR, FAILURE_INDICATOR, FAILURE_INDICATOR};
uint8_t ackMsg[17];
uint8_t ackLen = 17;
uint8_t data[RH_RF95_MAX_MESSAGE_LEN] = { 0 };
bool timeWindowOpen  = false;
sensors_event_t humi, temp;
/*
 *-------------------------------------------------------------------------*
 * Add any variables that are need to store sensor data here.              *
 * The order of variables are as follows: int->float->char->string->bool.  *
 * Always decalre and initialise your variables with a default value.      *
 * You do not need to intialise any custom datatype with a default value.  *
 * Use camel case for variable names: "int myVariable = 0;".               *
 *-------------------------------------------------------------------------*
*/

RH_RF95 rfm95(RFM95_CS, RFM95_IRQ);                                 // * Declare RFM95 radio object.
RTC_DS3231 rtc_module;                                              // * Declare and initialise the DS3231 setting I2C address to 0x68.
DFRobot_SGP40 sgp40(&Wire);                                         // * Declare SGP40 sensor object.
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();                            // * Decalre SHTC3 sensor object.
Adafruit_BMP3XX bmp388;                                             // * Decalre BMP388 sensor object.

/*
 *---------------------------------------------*
 * Add any object decalrations here.           *
 * Name of object should be in all lower case. *
 *---------------------------------------------*
*/

void setup() {
  Serial.begin(115200); // Intialise serial communication.
  pinMode(RFM95_RST, OUTPUT); // Set RFM95 reset pin to output.
  pinMode(UVSensorPin, OUTPUT);
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

  while (!rtc_module.begin()) { // Check if the clock module is responding.
    Serial.println("Error: DS3231 not detected!");
    Serial.flush();
    digitalWrite(FAILURE_LED_PIN, HIGH);
    delay(500);
    digitalWrite(FAILURE_LED_PIN, LOW);
    delay(500);
  }

  if (rtc_module.lostPower()) { // Check if the clock module has lost power and reset the time.
    Serial.println("RTC has lost power, resetting the time!");
    rtc_module.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  Serial.println("DS3231 Initialised!");

  /*
   *------------------------------------------------------------------------*
   * Add the initialisation of any modules here.                            *
   * Use a while loop to check for the intiialisation status as done above. *
   * Use a delay of 1s when retrying initialisation.                        *
   *------------------------------------------------------------------------*
  */
  delay(500);
}

void loop()
{
  // Get the current unix time
  unix_time = rtc_module.now().unixtime();

  // Check if it's time to send a message or if the time window is open
  if (timeWindowOpen || messageStatus(unix_time))
  {
    // Calculate the timestamp for message identification
    id_timestamp = round(unix_time / 60.0) * 60;
    
    // Read sensor data
    shtc3.getEvent(&humi, &temp);
    if (!bmp388.performReading())
    {
      Serial.println("Error: Failed to perform BMP388 reading!");
    }

    // Create the message packet for transmission
    CreateMessage();

    // Send the message packet via LoRa radio
    rfm95.send(data, 48); // data format -> [val1 *4, val2 *4, val3 *4, val4 *4, val5 *4, val6 *4, id_timestamp * 4, device_hash * 16] Each element is an octet 
    rfm95.waitPacketSent();


    if (rfm95.recv(ackMsg, &ackLen)) 
    {
      // Verify the integrity of the received acknowledgement packet
      if (acknowledgementPacketVerification(ackMsg))
      {
        // Process acknowledgement based on its content
        if (ackMsg[16] == SUCCESS_INDICATOR || ackMsg[16] == DUPLICATE_INDICATOR)
        {
          removeExpectedTimestamp(id_timestamp);
        }
        else if (ackMsg[16] == FAILURE_INDICATOR)
        {
          // If transmission failed, retry or close time window after maximum retries
          if (!timeWindowOpen)
          {
            timeWindowOpen = true;
          }
          else if (++retry_count == 3)
          {
            retry_count = 0;
            timeWindowOpen = false;
          }
        }
        // Send acknowledgement back to base station
        sendAcknowledgement(); // **Modify sendAcknowledgement to include device_hash and failure/ignore status**//
      }
      else
      {
        sendAcknowledgement(); // **Modify sendAcknowledgement to include device_hash and failure/ignore status**//
      }
    }

    delay(60000); // Wait a minute before proceeding

  }
  else
  {
    // Put the device into idle state temporarily
    rfm95.sleep();
    delay(ceil(unix_time / 900.0)*900 - unix_time);
  }

}

/*
 * Function: CreateMessage
 * ------------------------
 * 
 * Constructs the message packet for transmission.
 * 
 * This function constructs the message packet by copying sensor values, generating
 * device authentication hash, copying timestamps, and generating message hash.
 */
void CreateMessage()
{
  float humidity = humi.relative_humidity;
  float temperature = temp.temperature;
  float pressure = (float)(bmp388.pressure / 100.0);
  float altitude = (float)bmp388.readAltitude(SEA_LEVEL_PRESSURE_HPA);
  float UVIndex = calcUVIndex();
  float airQuality = sgp40.getVoclndex();
  memcpy(data, &humidity, sizeof(float));
  memcpy(data + sizeof(float), &temperature, sizeof(float));
  memcpy(data + 2 * sizeof(float), &pressure, sizeof(float));
  memcpy(data + 3 * sizeof(float), &altitude, sizeof(float));
  memcpy(data + 4 * sizeof(float), &UVIndex, sizeof(float));
  memcpy(data + 5 * sizeof(float), &airQuality, sizeof(float));
  // Calculate the current timestamp rounded to the nearest TIME_WINDOW second for synchronization.
  uint32_t current_timestamp = round(unix_time / TIME_WINDOW) * TIME_WINDOW;
  // Construct the expected device hash string using ALLOWED_MAC, UNIX time, PASSKEY.
  String expected_string = MAC_ADDRESS + ':' + String(current_timestamp) + ':' + PASSKEY;
  for (int i = 0; i < 24; i++)
  {
    expected_string += '|' + String(data[i]);
  }
  // Generate the expected device hash for comparison
  uint8_t *expected_hash_buffer = generateHash(expected_string.c_str());
  memcpy(data  + 6 * sizeof(float), expected_hash_buffer, 16);
  // Free allocated memory to store the hash
  delete[] expected_hash_buffer;
  memcpy(data + 10 * sizeof(float), &unix_time, 4);
  memcpy(data + 11 * sizeof(float), &id_timestamp, 4);
}


/*
 * Function: generateHash
 * -----------------------
 *
 * Generates an MD5 hash for the given data and returns it as an array of uint8_t.
 * 
 * Parameters:
 *   data: A pointer to a constant character array containing the data to be hashed.
 * 
 * Returns:
 *   uint8_t*: A pointer to an array of uint8_t containing the generated hash.
 */
uint8_t* generateHash(const char* data)
{
  // Generate MD5 hash (128 bits)
  unsigned char* hash = MD5::make_hash(const_cast<char*>(data));
  // Create a buffer to split 128 bits into 16 octets
  uint8_t* hashBuffer =  new uint8_t[HASH_SIZE];
  for (int i = 0; i < HASH_SIZE; i++)
  {
    // Extract an octet and convert it to uint8_t
    hashBuffer[i] = static_cast<uint8_t>(hash[i]); 
  } 
  // Free allocated memory to store the hash
  free(hash);
  return hashBuffer;
}

/*
 * Function: messageStatus
 * ------------------------
 *
 * Checks if the current time is divisible by 15 minutes and updates the expected_messages_timestamps array accordingly.
 * 
 * Parameters:
 *   unix_time: An unsigned 32-bit integer representing the current UNIX time.
 * 
 * Returns:
 *   bool: True if the time is divisible by 15 minutes, otherwise false.
 */
bool messageStatus(uint32_t unix_time)
{
  // Checks if time is divisible by 15 minutes
  if ((int)(round(unix_time / 60.0) * 60.0) % 900 == 0)
  { 
    // Check if expected_messages_timestamps has been allocated
    if (expected_messages_timestamps != nullptr)
    {
      // Calculate number of elements in expected_messages_timestamps
      messages_count = sizeof(expected_messages_timestamps) / sizeof(expected_messages_timestamps[0]); 
    }
    // Create a buffer to store the modified array
    uint32_t* buffer = new uint32_t[messages_count +1];
    // Copy contents of expected_messages_timestamps to the buffer
    for (int i = 0; i < messages_count; i++)
    {
      buffer[i] = expected_messages_timestamps[i];  
    }
    // Add unix_time to the buffer
    buffer[messages_count] = unix_time;
    // Free allocated memory to store the old array
    delete[] expected_messages_timestamps;
    // Change expected_messages_timestamps to point to the new buffer
    expected_messages_timestamps = buffer;
    return true; 
  }
  // Returns false if time isn't divisble by 15 minutues
  return false;
}

/*
 * Function: sendAcknowledgement
 * -----------------------------
 * 
 * Sends acknowledgement packets to the base station.
 * 
 * This function sends acknowledgement packets to the base station in case of successful
 * reception, duplication, or failure of message transmission.
 */
void sendAcknowledgement()
{
  int i = 1;
  do
  {
    rfm95.send(expected, 5);
    rfm95.waitPacketSent();
    i++;
  }
  while (i<=3);
}
/*
 * Function: removeExpectedTimestamp
 * ---------------------------------
 * 
 * Removes the specified timestamp from the array of expected message timestamps.
 * 
 * Parameters:
 *   timestamp: The timestamp to be removed.
 */
void removeExpectedTimestamp(uint32_t timestamp)
{
  for (int i = 0; i < messages_count; i++)
  { 
    if (timestamp == expected_messages_timestamps[i])
    {
      // Shift all elements after the timestamp to the left
      for (int j = i; j < messages_count - 1; j++)
      {
        expected_messages_timestamps[j] = expected_messages_timestamps[j + 1];
      }
      // Decrease the count of elements
      messages_count--; 
      return;
    }
  }
}

/*
 * Function: calcUVIndex
 * ----------------------
 * 
 * Calculates the UV index based on the analog value read from the UV sensor.
 * 
 * Returns:
 *   float: The calculated UV index.
 */
float calcUVIndex()
{
  int readVal = analogRead(UVSensorPin);
  float UVIndex = 0.0;

  // Calculate the UV index based on the anlogue value
  if (readVal <= 10)
  {
    UVIndex = 0.0;
  }
  else if (readVal <= 25.782)
  {
    UVIndex = (0.02778 * readVal) - 0.2778;
    UVIndex = (float)round(UVIndex);
  }
  else
  {
    UVIndex = (0.05147 * readVal) - 1.327;
    UVIndex = (float)round(UVIndex);
  }
  return UVIndex;
}

/*
 * Function: acknowledgementPacketVerification
 * -------------------------------------------
 * 
 * Verifies the integrity of the acknowledgement packet received from the base station.
 * This function reconstructs the MD5 hash string based on the received acknowledgement packet
 * and compares it with the expected hash string to determine if the packet is valid.
 * 
 * Parameters:
 *   ackMsg: A pointer to an array of uint8_t containing the acknowledgement packet.
 * 
 * Returns:
 *   bool: True if the acknowledgement packet is valid, otherwise false.
 */ 
bool acknowledgementPacketVerification(uint8_t* ackMsg)
{
  
  // Calculate the current timestamp rounded to the nearest TIME_WINDOW second for synchronization.
  uint32_t current_timestamp = round(rtc_module.now().unixtime() / TIME_WINDOW) * TIME_WINDOW;
  // Construct ackDevice hash string using BASE_MAC_ADDR, UNIX time, BASE_PASSKEY, and ackIndicator.
  String ackDevice_hash_string = BASE_MAC_ADDR + ':' + String(current_timestamp) + ':' + BASE_PASSKEY + '_'+ ackMsg[16]; 
  // Generate ackDevice hash.
  uint8_t *ackDevice_hash_buffer = generateHash(ackDevice_hash_string.c_str()); 
  String expected_hash = reconstructHash(ackDevice_hash_buffer);
  String received_hash = reconstructHash(ackMsg);
  // Free allocated memory to store ackDevice_hash_buffer.
  delete[] ackDevice_hash_buffer;
  return expected_hash == received_hash;
}


/*
 * Function: reconstructHash
 * ---------------------------
 *
 * Reconstructs an MD5 hash string from the given hash buffer.
 * 
 * Parameters:
 *   hashBuffer: A pointer to an array of uint8_t containing the hash buffer (16 elements, each element is an octet).
 * 
 * Returns:
 *   String: The reconstructed MD5 hash string.
 */
String reconstructHash(uint8_t* hashBuffer)
{
  String hash = "";
  for (int i = 0; i < HASH_SIZE; i++)
  {
    hash += String(hashBuffer[i], HEX);
  }
  return hash;
}








