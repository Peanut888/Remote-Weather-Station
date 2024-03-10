/*
 * Programmer: Justin Lam
 * Date: 04/03/24
 * Description: Receive humidity and temeprature data via Lo-Ra radio.
*/

#include <SPI.h>
#include <RH_RF95.h>
/*
 *--------------------------------------------*
 * Add any necessary library inclusions here. *
 *--------------------------------------------*
*/

#define RFM95_IRQ 3 // RFM95 interrupt pin.
#define RFM95_RST 4 // RFM95 reset pin.
#define RFM95_CS 8 // RFM95 slave select pin.
#define RFM95_FREQ 433.0 // Radio frequency of 433Hz.
/*
 *---------------------------------------------------------------------------------*
 * Add preprocessor definitions here.                                              *
 * All definitions must be named in all caps separated by an underscore character. *
 * Example: "#define MY_DEFINITION 0".                                             *
 *---------------------------------------------------------------------------------*
*/

void parse(int length, float *data, char *buffer); // Parse the data from a string to arry of float.
/*
 *------------------------------------------------------------------------*
 * Add any function forward declarations here.                            *
 * Function names should be in camel case: "void myFunction();".          *
 * The order of arguments are as follows: int->float->char->string->bool. *
 *------------------------------------------------------------------------*
 */

uint8_t dataLen = 0;
float values[4] = {0.0};
char data[RH_RF95_MAX_MESSAGE_LEN] = { '\0' };
char acknowledge[] = "Data Ok!";
/*
 *-------------------------------------------------------------------------*
 * Add any variables that are need to store sensor data here.              *
 * The order of variables are as follows: int->float->char->string->bool.  *
 * Always decalre and initialise your variables with a default value.      *
 * You do not need to intialise any custom datatype with a default value.  *
 * Use camel case for variable names: "int myVariable = 0;".               *
 *-------------------------------------------------------------------------*
*/

RH_RF95 rfm95(RFM95_CS, RFM95_IRQ); // Decalre RFM95 radio object.
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
  rfm95.setTxPower(23, false);
  Serial.println("RFM95 successfully intitialised!"); // Set the transmission power level.
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
  // Listen for any incoming packets.
  if (rfm95.available()) { // If received print the data received and send acknowledgement packet.
    dataLen = sizeof(data);

    // Parse and print the data received on the serial monitor.
    if (rfm95.recv((uint8_t *)data, &dataLen)) {
      parse(4, values, data);
      Serial.print(values[0]);
      Serial.println("%");
      Serial.print(values[1]);
      Serial.println("*C");
      Serial.print(values[2]);
      Serial.println("hPa");
      Serial.print(values[3]);
      Serial.println("m");
      Serial.println();
      rfm95.send((uint8_t *)acknowledge, 9); // Send acknowledgement packet.
      rfm95.waitPacketSent(); // Wait until the acknowledgement packet has been sent.
    } // Print failure message if data reception has failed.
    else {
      Serial.println("Failed to receive data!");
    }
  }
}

// Parse the data from string to array of floats.
void parse(int length, float *data, char *buffer) {
  size_t len = strlen(buffer); // Not needed.
  char parseStr[30] = {'\0'}, tempStr[RH_RF95_MAX_MESSAGE_LEN] = {'\0'}; // Temporary strings.

  strcpy(tempStr, buffer); // Copy the data string from the incoming buffer.
  strcpy(parseStr, strtok(tempStr, " ")); // Read the first value.
  data[0] = atof(parseStr); // Convert the value from ASCII to float and store in array.

  for (int i = 1; i < length - 1; i++) // Convert the remaning data minus the last.
  {
    strcpy(parseStr, strtok(NULL, " "));
    data[i] = atof(parseStr);
  }

  // Convert the last data.
  strcpy(parseStr, strtok(NULL, "\0"));
  data[length - 1] = atof(parseStr);
}
/*
 *-----------------------------------------------*
 * All function definitions are to be made here. *
 *-----------------------------------------------*
*/