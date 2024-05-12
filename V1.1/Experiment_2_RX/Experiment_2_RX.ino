/*
 * Programmer: Justin Lam, Alhussain Albusaidi
 * Date: 11/05/2024
 * Project: Lo-Ra Weather Data Transmission
 * Description: This project involves the implementation of a Lo-Ra based system
 *              for wirelessly receiving humidity and temperature data. The system 
 *              ensures data integrity and authenticity through MD5 hashing and 
 *              device verification mechanisms. It also handles duplicates and 
 *              measures latency to maintain data accuracy and reliability.

*/

#include <SPI.h>
#include <RH_RF95.h>
#include "MD5.h"
#include <RTClib.h> 
#include <math.h>

/*
 *--------------------------------------------*
 * Add any necessary library inclusions here. *
 *--------------------------------------------*
*/

#define RFM95_IRQ 3                                                             // * RFM95 interrupt pin.
#define RFM95_RST 4                                                             // * RFM95 reset pin.
#define RFM95_CS 8                                                              // * RFM95 slave select pin.
#define RFM95_FREQ 433.0                                                        // * Radio frequency of 433Hz.
#define FAILURE_LED_PIN  28                                                     // * Failure to transmit debug LED pin.

#define HASH_SIZE 16                                                            // * 16 octets (8 bits) are required to store 128 bits
#define ALLOWED_MAC "33:b7:ef:28:63:74"                                         // * Example mac, redefine as an array if more than is required
#define PASSKEY "qw@jZTB8vnY!"                                                  // * Example passkey, redefine as an array if more than is required
#define BASE_MAC_ADDR "2b:85:0f:d5:0f:21"                                       // * Example mac, replace with base station mac address
#define BASE_PASSKEY "14M0Itj6Q+C8)p$2"                                         // * Example passkey

#define MESSAGE_SIZE 44                                                         // * Total number of octets in data[]
#define DEVICE_HASH_LOWER_INDEX 28                                              // * First device hash element in data[]
#define ID_TIMESTAMP_LOWER_INDEX 24                                             // * First id timestamp elementin data[]
#define ID_TIMESTAMP_UPPER_INDEX 27                                             // * Last id timestamp element in data[]


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
String reconstructHash(uint8_t* hashBuffer);                                    // * Reconstruct an MD5 hash string from the given hash buffer.
bool messageStatus(uint32_t unix_time);                                         // * Check if it's time to send a message based on the current UNIX time.
bool isDuplicate(uint8_t* data);                                                // * Check if the received data contains a duplicate timestamp and handle it if found.
bool validateData(uint8_t* data);                                               // * Validate the integrity of the received data packet by comparing the received message hash with the expected message hash.
bool receiveAcknowledgement();                                                  // * Listens for an incoming acknowledgment packet and compares it with the expected acknowledgment.
void acknowledgementPacket(int ackStatus);
void writeToSdcard(uint8_t* data);

/*
 *------------------------------------------------------------------------------*
 * Add any function forward declarations here.                                  *
 * Function names should be in camel case: "void myFunction();".                *
 * The order of arguments are as follows: void->int->float->char->string->bool. *
 *------------------------------------------------------------------------------*
 */
int retry_count = 0;
int messages_count = 0;
uint32_t unix_time = 0;
uint32_t* expected_messages_timestamps = nullptr;                               // * Pointer to an array storing timestamps of expected messages.
uint8_t dataLen = 40;
float values[5] = {0.0};
char data[RH_RF95_MAX_MESSAGE_LEN] = { '\0' };                                  // * RH_RF95_MAX_MESSAGE_LEN = 251 octets, available space for data is 250 octet. Each element is an octet
bool timeWindowOpen  = false;
/*
 *-------------------------------------------------------------------------*
 * Add any variables that are need to store sensor data here.              *
 * The order of variables are as follows: int->float->char->string->bool.  *
 * Always decalre and initialise your variables with a default value.      *
 * You do not need to intialise any custom datatype with a default value.  *
 * Use camel case for variable names: "int myVariable = 0;".               *
 *-------------------------------------------------------------------------*
*/

RH_RF95 rfm95(RFM95_CS, RFM95_IRQ);                                             // * Declare RFM95 radio object.
RTC_DS3231 rtc_module;                                                          // * Declare and initialise the DS3231 setting I2C address to 0x68.

/*
 *---------------------------------------------*
 * Add any object decalrations here.           *
 * Name of object should be in all lower case. *
 *---------------------------------------------*
*/


void setup()
{
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
  while (!rfm95.init())
  {
    Serial.println("Error: Failed to initialise RFM95!");
    Serial.println("Retrying...");
    delay(1000);
  }

  // Set the frequency of the RFM95 radio.
  while (!rfm95.setFrequency(RFM95_FREQ))
  {
    Serial.println("Error: Failed to set frequency to to 433HZ!");
    Serial.println("Retrying...");
    delay(1000);
  }

  // Print the frequency to serial monitor.
  Serial.print("Frequency set to ");
  Serial.print(RFM95_FREQ);
  Serial.println("Hz.");
  rfm95.setTxPower(23, false);
  Serial.println("RFM95 successfully intitialised!"); // Set the transmission power level.


  while (!rtc_module.begin()) // Check if the clock module is responding.
  {
    Serial.println("Error: DS3231 not detected!");
    Serial.flush();
    digitalWrite(FAILURE_LED_PIN, HIGH);
    delay(500);
    digitalWrite(FAILURE_LED_PIN, LOW);
    delay(500);
  }

  if (rtc_module.lostPower()) // Check if the clock module has lost power and reset the time.
  {
    Serial.println("RTC has lost power, resetting the time!");
    rtc_module.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

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
  unix_time = rtc_module.now().unixtime(); // Get the current unix time
  if (timeWindowOpen || messageStatus(unix_time))
  {
    if (rfm95.available())
    { 
      if (rfm95.recv((uint8_t *)data, &dataLen)) // data format -> [val1 *4, val2 *4, val3 *4, val4 *4, val5 *4, val6 *4, id_timestamp * 4, device_hash * 16] Each element is an octet 
      { 
        if (validateData((uint8_t *)data))
        {
            if (!isDuplicate((uint8_t *)data))
            {
              // Send acknowledgement packet, success
              acknowledgementPacket(ackSuccess);
              receiveAcknowledgement();
              //  Save data to sdcard
              writeToSdcard((uint8_t *)data); // Placeholder function
  
            }
            else
            {
              // Send acknowledgement packet, ignore
              acknowledgementPacket(ackDuplicate);
              receiveAcknowledgement();
  
            }
        }
        else
        {
          // Send acknowledgement packet, failure
          acknowledgementPacket(ackFailure);
          receiveAcknowledgement();
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
      }
      else
      {
        Serial.println("Failed to receive data!");
      }
    }
    delay(60000);
  }
  else
  {
    // Put the device into idle state temporarily
    rfm95.sleep();
    delay(ceil(unix_time / 900.0)*900 - unix_time);
  }
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
  unsigned char* hash = MD5::make_hash(const_cast<char*>(data)); // Generate MD5 hash (128 bits)

  uint8_t* hashBuffer =  new uint8_t[HASH_SIZE]; // Create a buffer to split 128 bits into 16 octets

  for (int i = 0; i < HASH_SIZE; i++)
  {
    hashBuffer[i] = static_cast<uint8_t>(hash[i]);  // Extract an octet and convert it to uint8_t
  } 
  
  free(hash); // Free allocated memory to store the hash

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
  if ((int)(round(unix_time / 60.0) * 60.0) % 900 == 0) // Returns true if time is divisible by 15 minutes
  { 
    if (expected_messages_timestamps != nullptr) // Check if expected_messages_timestamps has been allocated
    {
      messages_count = sizeof(expected_messages_timestamps) / sizeof(expected_messages_timestamps[0]);  // Calculate number of elements in expected_messages_timestamps
    }
    uint32_t* buffer = new uint32_t[messages_count +1]; // Create a buffer to store the modified array
    for (int i = 0; i < messages_count; i++)
    {
      buffer[i] = expected_messages_timestamps[i];  // Copy contents of expected_messages_timestamps to the buffer
    }
    buffer[messages_count] = unix_time;    // Add unix_time to the buffer
    delete[] expected_messages_timestamps; // Free allocated memory to store the old array
    expected_messages_timestamps = buffer; // Change expected_messages_timestamps to point to the new buffer
    return true; 
  }
  return false; // Returns false if time isn't divisble by 15 minutues
}

/*
 * Function: isDuplicate
 * ----------------------
 *
 * Checks if the provided timestamp already exists in the expected_messages_timestamps array.
 * If a duplicate is found, it removes the duplicate entry from the array.
 * 
 * Parameters:
 *   data: A pointer to an array of uint8_t containing the data packet, with the timestamp located at indices 21 to 23.
 * 
 * Returns:
 *   bool: True if a duplicate is found and removed, otherwise false.
 * 
 * Details:
 *   This function iterates over the elements in the expected_messages_timestamps array to check if the provided timestamp exists.
 *   If the timestamp exists in the array, it indicates that the data is not a duplicate, and it removes the timestamp entry from the array.
 *   If the timestamp does not exist in the array, it means it has already been removed, indicating a duplicate data packet.
 */
bool isDuplicate(uint8_t* data)
{
  uint32_t timestamp = 0;
  for(int i = ID_TIMESTAMP_UPPER_INDEX; i >= ID_TIMESTAMP_LOWER_INDEX; i--)
  {
    timestamp <<= 8; // Shift the current value of timestamp 8 bits to the left
    timestamp |= data[i]; // Add the next byte to the timestamp
  }
  // Loop over all expected_messages_timestamps
  for (int i = 0; i < messages_count; i++)
  { 
    if (timestamp == expected_messages_timestamps[i])
    {
      // Shift all elements after the timestamp to the left
      for (int j = i; j < messages_count - 1; j++)
      {
        expected_messages_timestamps[j] = expected_messages_timestamps[j + 1];
      }
      messages_count--; // Decrease the count of elements
      return false; // Returns false if data is not duplicate
    }
  }
  return true; // Returns true if data is duplicate
}

/*
 * Function: validateData
 * ------------------------
 *
 * Validates the integrity of the received data packet by comparing the received message hash with the expected hash.
 * 
 * Parameters:
 *   data: A pointer to an array of uint8_t containing the received data packet.
 * 
 * Returns:
 *   bool: True if the received message hash matches the expected hash, indicating data integrity, otherwise false.
 */
bool validateData(uint8_t* data)
{
  uint32_t current_timestamp = round(unix_time / TIME_WINDOW) * TIME_WINDOW; // Calculate the current timestamp rounded to the nearest TIME_WINDOW second for synchronization purposes.

  // Extract the subset of elements (message hash) from data array
  uint8_t message_hash_array[HASH_SIZE]; 
  for (int i = 0; i < HASH_SIZE; i++)
  {
    message_hash_array[i] = data[i + DEVICE_HASH_LOWER_INDEX]; // Start from the 24th element and copy 16 elements
  }
  String received_message_hash = reconstructHash(message_hash_array); // Reconstruct the received message hash 


  String expected_message_string = ALLOWED_MAC + ':' + String(current_timestamp) + ':' + PASSKEY; // Construct the expected device hash string using ALLOWED_MAC, UNIX time, PASSKEY.
  for (int i = 0; i <= ID_TIMESTAMP_UPPER_INDEX; i++)
  {
    expected_message_string += '|' + String(data[i]); // Start from the 24th element and copy 16 elements
  }
  uint8_t *expected_message_hash_buffer = generateHash(expected_message_string.c_str()); // Generate the expected device hash for comparison
  String expected_message_hash = reconstructHash(expected_message_hash_buffer);  // Reconstruct the expected device hash
  delete[] expected_message_hash_buffer; // Deallocate the memory allocated for the expected device hash buffer

  return received_message_hash == expected_message_hash;
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
String reconstructHash(uint8_t* hashBuffer) // **REVIST THIS FUNCTION (USE MEMCPY)**//
{
  String hash = "";
  for (int i = 0; i < HASH_SIZE; i++)
  {
    hash += String(hashBuffer[i], HEX);
  }
  return hash;
}


/*
 * Function: acknowledgementPacket
 * -------------------------------
 * 
 * Constructs and sends an acknowledgment packet over the RFM95 radio module.
 * 
 * This function constructs an acknowledgment packet containing a device hash and an acknowledgment status indicator,
 * then sends the packet over the RFM95 radio module. The acknowledgment status indicator can be one of the following:
 * - ackSuccess: Indicates successful reception and processing of the data packet.
 * - ackFailure: Indicates failure in processing the data packet.
 * - ackDuplicate: Indicates that the received data packet is a duplicate.
 * 
 * Parameters:
 *   ackStatus: An integer representing the acknowledgment status. It can have one of the following values:
 *              - ackSuccess: 0, indicating successful acknowledgment.
 *              - ackFailure: 1, indicating acknowledgment of failure.
 *              - ackDuplicate: 2, indicating acknowledgment of a duplicate.
 * 
 * Returns:
 *   None
 * 
 * Details:
 *   - The function constructs the acknowledgment packet using the following steps:
 *     - Determines the current timestamp rounded to the nearest TIME_WINDOW second for synchronization.
 *     - Constructs the acknowledgment device hash string using BASE_MAC_ADDR, UNIX time, BASE_PASSKEY, and acknowledgment status.
 *     - Generates the acknowledgment device hash.
 *     - Copies 16 bytes from the acknowledgment device hash buffer to the acknowledgment packet.
 *     - Sets the last byte of the acknowledgment packet to indicate the acknowledgment status.
 *   - Finally, the function sends the acknowledgment packet over the RFM95 radio module. 
 */
void acknowledgementPacket(int ackStatus)
{
  uint8_t acknowledgement_message[17]; // acknowledgement_message format -> [ackDevice_hash *16, ackStatus]

  // Set the last element of acknowledgement_message to indicate the ackStatus
  switch (ackStatus)
  {
    case ackSuccess:
      acknowledgement_message[16] = SUCCESS_INDICATOR;
      break;
    case ackFailure:
      acknowledgement_message[16] = FAILURE_INDICATOR;
      break;
    case ackDuplicate:
      acknowledgement_message[16] = DUPLICATE_INDICATOR;
      break;
  }
  // Calculate the current timestamp rounded to the nearest TIME_WINDOW second for synchronization.
  uint32_t current_timestamp = round(rtc_module.now().unixtime() / TIME_WINDOW) * TIME_WINDOW;
  // Construct ackDevice hash string using BASE_MAC_ADDR, UNIX time, BASE_PASSKEY, and ackIndicator.
  String ackDevice_hash_string = BASE_MAC_ADDR + ':' + String(current_timestamp) + ':' + BASE_PASSKEY + '_'+ acknowledgement_message[16]; 
  // Generate ackDevice hash.
  uint8_t *ackDevice_hash_buffer = generateHash(ackDevice_hash_string.c_str()); 
  // Copy 16 bytes from ackDevice_hash_buffer to acknowledgement_message.
  memcpy(acknowledgement_message, ackDevice_hash_buffer, HASH_SIZE); 
  // Free allocated memory to store ackDevice_hash_buffer.
  delete[] ackDevice_hash_buffer;

  // Send acknowledgement packet over the RFM95 radio module
  rfm95.send(acknowledgement_message, 17);
  rfm95.waitPacketSent();
}


/*
 * Function: receiveAcknowledgement
 * -----------------------------
 * 
 * Receives an acknowledgement packet over the RFM95 radio module and compares it with the expected acknowledgment.
 * 
 * This function waits for an acknowledgement packet to be available within a specified timeout period using the RFM95 module.
 * Upon receiving the packet, it compares each byte of the received buffer with the corresponding byte of the expected acknowledgment.
 * If all bytes match, it returns true, indicating successful reception of the expected acknowledgment.
 * If any byte does not match or if no data is received within the timeout period, it returns false.
 * 
 * Returns:
 *   bool: True if the received acknowledgment matches the expected acknowledgment, otherwise false.
 * 
 */
bool receiveAcknowledgement()
{ 
  uint8_t buffer[5];
  uint8_t len = 5;
  const uint8_t expected[] = {SUCCESS_INDICATOR, SUCCESS_INDICATOR, DUPLICATE_INDICATOR, FAILURE_INDICATOR, FAILURE_INDICATOR};
  int j = 1;
  bool false_ack;
  do
  {
    if (rfm95.waitAvailableTimeout(6000) && rfm95.recv(buffer, &len))
    {
      false_ack = false;  
      for (int i = 0; i < 5; i++)
      {
        if (expected[i] != buffer[i])
        {
          false_ack = true;
          break; 
        }
      }
      if(false_ack)
      {
        continue;
      }
      return true;
    }
    else
    {
      continue;
    }
  }
  while (j <= 3);
  return false;
}

void writeToSdcard(uint8_t* data)
{
  float humidity, temperature, pressure, altitude, UVIndex, airQuality;
  uint32_t timestamp;
  memcpy(&humidity, data, sizeof(float)); 
  memcpy(&temperature, data + sizeof(float), sizeof(float)); 
  memcpy(&pressure, data + 2 * sizeof(float), sizeof(float)); 
  memcpy(&altitude, data + 3 * sizeof(float), sizeof(float)); 
  memcpy(&UVIndex, data + 4 * sizeof(float), sizeof(float)); 
  memcpy(&airQuality, data + 5 * sizeof(float), sizeof(float)); 
  memcpy(&timestamp, data + 6 * sizeof(float), 4);

}
/*
 *-----------------------------------------------*
 * All function definitions are to be made here. *
 *-----------------------------------------------*
*/