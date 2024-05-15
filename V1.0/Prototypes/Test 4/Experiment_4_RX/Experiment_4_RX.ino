/*
 * Programmer: Justin Lam
 * Date: 03/05/24
 * Description: Receive humidity and temeprature, pressure, altitude, and air quality, UV index data via Lo-Ra radio, and time stamp the data
 *              and display data onto an OLED display.
*/

#include <SPI.h>
#include <RH_RF95.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
/*
 *--------------------------------------------*
 * Add any necessary library inclusions here. *
 *--------------------------------------------*
*/

#define RFM95_IRQ 3 // RFM95 interrupt pin.
#define RFM95_RST 4 // RFM95 reset pin.
#define RFM95_CS 8 // RFM95 slave select pin.
#define SCK_PIN 24 // SPI SCK pin.
#define MOSI_PIN 22 // SPI MOSI pin.
#define CS_PIN 12 // OLED display slave select pin.
#define RST_PIN 6 // OLED display reset pin.
#define DC_PIN 5 // OLED display data pin.

#define RFM95_FREQ 433.0 // Radio frequency of 433Hz.
#define SCREEN_WIDTH 128 // OLED display screen width.
#define SCREEN_HEIGHT 96 // OLED display screen height.
// Colours
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
/*
 *---------------------------------------------------------------------------------*
 * Add preprocessor definitions here.                                              *
 * All definitions must be named in all caps separated by an underscore character. *
 * Example: "#define MY_DEFINITION 0".                                             *
 *---------------------------------------------------------------------------------*
*/

void parse(int length, float *data, char *buffer); // Parse the data from a string to arry of float.
void displayClear(); // Clear the display.
void displayText(uint8_t size, uint16_t colour, uint16_t x, uint16_t y, String text); // Display a string on the OLED.
void displayData(uint8_t size, uint16_t colour, uint16_t x, uint16_t y); // Display the data from the data array on the OLED.
void pwrOLED(); // Power on or off the OLED module.
/*
 *------------------------------------------------------------------------*
 * Add any function forward declarations here.                            *
 * Function names should be in camel case: "void myFunction();".          *
 * The order of arguments are as follows: int->float->char->string->bool. *
 *------------------------------------------------------------------------*
 */

const int buttonPin = 11;
uint8_t dataLen = 0;
float values[6] = {0.0};
char data[RH_RF95_MAX_MESSAGE_LEN] = { '\0' };
char acknowledge[] = "Data Ok!";
String timeDate = "";
bool OLEDPwrState = false;
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

RH_RF95 rfm95(RFM95_CS, RFM95_IRQ); // Decalre RFM95 radio object.
RTC_DS3231 rtc; // Declare the DS3231 RTC object.
Adafruit_SSD1351 oled = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN, RST_PIN); // Decalre the SSD1351 OLED driver object.
/*
 *---------------------------------------------*
 * Add any object decalrations here.           *
 * Name of object should be in all lower case. *
 *---------------------------------------------*
*/

void setup() {
  Serial.begin(115200); // Intialise serial communication.
  pinMode(RFM95_RST, OUTPUT); // Set RFM95 reset pin to output.
  pinMode(buttonPin, INPUT); // Set the push button pin to input.
  // Reset the RFM95 radio.
  digitalWrite(RFM95_RST, HIGH);
  delay(20);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  attachInterrupt(digitalPinToInterrupt(buttonPin), pwrOLED, RISING); // Set up a hardware interrupt for the push button.

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

  // Initialise the OLED
  oled.begin();
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
    now = rtc.now();

    // Parse and print the data received on the serial monitor.
    if (rfm95.recv((uint8_t *)data, &dataLen)) {
      // Format string for time and date.
      timeDate = String(now.hour()) + ':' + String(now.minute()) + ':' + String(now.second()) + ' ' + String(now.day()) + '/' + String(now.month()) + '/' + String(now.year());
      parse(6, values, data); // Parse the data received.
      Serial.println(timeDate); // Print the time and the date to serial monitor.
      // Print the data to the serial monitor.
      Serial.print(values[0]);
      Serial.println("%");
      Serial.print(values[1]);
      Serial.println("*C");
      Serial.print(values[2]);
      Serial.println("hPa");
      Serial.print(values[3]);
      Serial.println("m");
      Serial.println(values[4]);
      Serial.println(values[5]);
      Serial.println();
      // Display the data on the OLED.
      displayClear();
      displayData(1, WHITE, 0, 0);
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

// Clear the display.
void displayClear() {
  oled.fillScreen(BLACK); // Turn all the pixels of the OLED off.
}

// Display a string on the OLED.
void displayText(uint8_t size, uint16_t colour, uint16_t x, uint16_t y, String text) {
  oled.setTextSize(size); // Set text size.
  oled.setTextColor(colour); // Set text colour.
  oled.setCursor(x, y); // Set text starting point.
  oled.println(text); // Print the text on the OLED.
}

// Display the data from the data array on the OLED.
void displayData(uint8_t size, uint16_t colour, uint16_t x, uint16_t y) {
  oled.setTextSize(size); // Set text size.
  oled.setTextColor(colour); // Set text colour.
  oled.setCursor(x, y); // Set text starting point.
  oled.println(timeDate); // Print the current time and date string.
  // Display all data received onto the OLED.
  oled.print("Humidity: ");
  oled.print(values[0]);
  oled.println("%");
  oled.print("Temperature: ");
  oled.print(values[1]);
  oled.println("*C");
  oled.print("Pressure: ");
  oled.print(values[2]);
  oled.println("hPa");
  oled.print("Altitude: ");
  oled.print(values[3]);
  oled.println("m");
  oled.print("VOC Index: ");
  oled.println(values[4]);
  oled.print("UV Index: ");
  oled.println(values[5]);
}

// Power on or off the OLED module.
void pwrOLED() {
  OLEDPwrState = !OLEDPwrState; // Toggle between the button states.
  oled.enableDisplay(OLEDPwrState); // Toggle the power of the OLED module.
  delay(10);
}
/*
 *-----------------------------------------------*
 * All function definitions are to be made here. *
 *-----------------------------------------------*
*/