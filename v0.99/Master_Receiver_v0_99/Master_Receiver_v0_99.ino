/*
Title: Remote Weather Station Master Receiver V0.99
Programmer: Justin Lam
Date: 28/02/23
Description: Master receiver for the remote weather station, reads data from transmitter and sends it to the receiver slave.
*/

#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define BLINK_COUNT 5
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3D

const int getDataButtonPin = 2; // Data request button pin.
const int blueLedPin = 3; // RGB blue pin.
const int greenLedPin = 5; // RGB green pin.
const int redLedPin = 6; // RGB red pin.
const int CEPin = 7; // nRF24 chip enable pin.
const int CSNPin = 8; // nRF24 chip select pin.
const int OLEDPowerButtonPin = 19; // OLED power button pin.
const byte address[2][6] = {"00001", "00002"}; // Define the address through which the radios will use to communicate.
int dataButtonState = 0; // Integer to store the current toggle state of the data request button.
int numRetry = 0;
bool isSuccess = false; // Transmission status.
bool ackSuccess = false; // Acknowledgement status.
bool powerButtonState = true; // Boolean value to store the state of the OLED power button.

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
Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Declare and initialise the SSD1306 setting I2C address 0x3D.
dataPacket data; // Declare the data packet named "data".
ackPacket received; // Declare the acknowledge packet named "received".
buttonPacket request; // Declare the button packet named "request".

void setup() {
  Serial.begin(9600); // Initialise serial communication.
  Wire.begin(); // Initialise I2C communication.

  /* Debug LED Initialisation */
  // Assign the LED pins as outputs.
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  // Turn off all the LEDs.
  digitalWrite(redLedPin, HIGH);
  digitalWrite(greenLedPin, HIGH);
  digitalWrite(blueLedPin, HIGH);

  // Not required
  /*
  #ifndef ESP8266
  while (!Serial) {
    blinkLEDDebug(255, 0, 0, 500, BLINK_COUNT);
  }
  #endif
  */

  /* SSD1306 Initialisation */
  //delay(100);
  
  if (!OLED.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) { // Check if the OLED display is responding.
    Serial.print("Error: SSD1306 not responding!");
    blinkLEDDebug(255, 0, 0, 500, BLINK_COUNT);
    LEDDebug(255, 0, 0, true);
    while(1);
  }
  else { // After initialisation clear the display buffer.
    OLED.clearDisplay();
    displayText(1, WHITE, 15, 28, "Initialising...");
    delay(1500);
  }

  /* nRF24 Initialisation */
  if (!radio.begin()) { // Check if the radio is responding.
    Serial.println("Error: nRF24 radio not responding!");
    Serial.println("Please check wiring...");
    displayText(1, WHITE, 0, 28, "nRF24 not responding!");
    blinkLEDDebug(255, 0, 0, 500, BLINK_COUNT);
    LEDDebug(255, 0, 0, true);
    while(1);
  }
  else {
    radio.openWritingPipe(address[1]); // Set the address for transmission.
    radio.openReadingPipe(1, address[0]); // Set the address for reception.
    radio.setPALevel(RF24_PA_LOW); // Set the power amplification of the radio to low.
    radio.enableAckPayload(); // Enable the acknowledgement payload.
    Serial.println("nRF24 Initialised!");
    displayText(1, WHITE, 0, 28, "nRF24 Initialised!");
    blinkLEDDebug(0, 255, 0, 200, BLINK_COUNT);
    delay(250);
  }

  /* Push-Button Initialisation */
  // Assign the push-button pins as inputs.
  pinMode(getDataButtonPin, INPUT);
  pinMode(OLEDPowerButtonPin, INPUT);
  // Set up hardware interrupts for each button.
  attachInterrupt(digitalPinToInterrupt(getDataButtonPin), requestData, RISING);
  attachInterrupt(digitalPinToInterrupt(OLEDPowerButtonPin), pwrOLED, RISING);

  displayText(1, WHITE, 0, 26, "All Initialised!");
  delay(500);
}

void loop() {
  if (!powerButtonState) { // Turn the OLED display on or off according to the OLED power button.
    OLED.clearDisplay();
    OLED.display();
  }
  else {
    displayData();
  }

  if (dataButtonState == 1) { // Send a request for the transmitter to send new data if request button is pushed.
    request.buttonState = true; // Request to send data is confirmed.
    radio.stopListening(); // Enable transmission.
    isSuccess = radio.write(&request, sizeof(buttonPacket)); // Send the request packet to the transmitter.
    delay(25);
    dataButtonState = 0; // Reset the request button state.
    numRetry = 0; // Reset the number of retries.
    radio.startListening(); // Enable receiving.
    checkTransmission(isSuccess, numRetry); // Check if the request packet was successfully sent.
    delay(25);
  }

  radio.startListening(); // Enable receiving.
  delay(10);

  if (radio.available()) { // Check to see if transmitter has sent data.
    radio.read(&data, sizeof(dataPacket)); // Read the data received.
    received.isAck = true; // The receiver has received a data packet.
    ackSuccess = radio.writeAckPayload(1, &received, sizeof(ackPacket)); // Send the acknowledgement to the transmitter.

    if (ackSuccess) { // If the acknowledgement packet was successfully sent, begin data logging.
      // Display the data on the serial monitor.
      blinkLEDDebug(255, 255, 0, 50, 10);
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
      
      if (data.rain == 0) { // Print yes of rain sensor returns a logic low.
        Serial.println("Rain: Yes");
      }
      else { // Print no if rain sensor returns a logic high.
        Serial.println("Rain: No");
      }

      displayData(); // Display the data on the OLED.
      Wire.beginTransmission(6); // Initialise I2C communication to the slave Arduino.
      Wire.write((byte *)&data, sizeof(dataPacket)); // Send the data via I2C to the slave Arduino.
      Wire.endTransmission(); // End the I2C communication with the slave Arduino.
      delay(25);
    }
    else { // If acknowledgement failed, display error message.
      Serial.println("Error: Payload acknowledgement failed!");
      blinkLEDDebug(255, 0, 0, 100, BLINK_COUNT);
    }

    received.isAck = false; // Reset the acknowledgement status.
  }
  else { // Stay on standby mode.
    blinkLEDDebug(0, 0, 255, 250, 1);
  }
}

void LEDDebug(int red, int green, int blue, bool status) { // Set RGB LED colour and turn on or off the LED.
  int redVal = 255 - red;
  int greenVal = 255 - green;
  int blueVal = 255 - blue;

  if (status) {
    analogWrite(redLedPin, redVal);
    analogWrite(greenLedPin, greenVal);
    analogWrite(blueLedPin, blueVal);
  }
  else {
    digitalWrite(redLedPin, HIGH);
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(blueLedPin, HIGH);
  }
}

void blinkLEDDebug(int red, int green, int blue, int time, int count) { // Set RGB LED colour and blink the LED.
  int redVal = 255 - red;
  int greenVal = 255 - green;
  int blueVal = 255 - blue;

  for (int i = 0; i < count; i++) {
    analogWrite(redLedPin, redVal);
    analogWrite(greenLedPin, greenVal);
    analogWrite(blueLedPin, blueVal);
    delay(time);
    digitalWrite(redLedPin, HIGH);
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(blueLedPin, HIGH);
    delay(time);
  }
}

void displayText(uint8_t size, uint16_t colour, uint16_t x, uint16_t y, String text) { // Display a string on the OLED.
  OLED.clearDisplay();
  OLED.setTextColor(colour);
  OLED.setTextSize(size);
  OLED.setCursor(x, y);
  OLED.println(text);
  OLED.display();
}

void displayData() { // Display the data on the OLED.
  OLED.clearDisplay();
  OLED.setTextSize(1);
  OLED.setTextColor(WHITE);
  OLED.setCursor(0,0);
  OLED.print("Temperature: ");
  OLED.print(data.temperature);
  OLED.println(" C");
  OLED.print("Humidity: ");
  OLED.print(data.humidity);
  OLED.println('%');
  OLED.print("Pressure: ");
  OLED.print(data.pressure);
  OLED.println("Pa");
  OLED.print("Altitude: ");
  OLED.print(data.altitude);
  OLED.println('m');
  OLED.print("VOC Index: ");
  OLED.println(data.indexVOC);
  OLED.print("UV Index: ");
  OLED.println(data.indexUV);
  OLED.print("Wind Speed: ");
  OLED.print(data.rpm);
  OLED.println("RPM");
  OLED.print("Rain: ");

  if (data.rain == 0) { // Print yes if rain sensor returns a logic low.
    OLED.println("Yes");
  }
  else { // Print no if rain sensor returns a logic high.
    OLED.println("No");
  }

  OLED.display();
}

void requestData() { // On request button being pushed, set the state to 1.
  dataButtonState = 1;
}

void pwrOLED() { // On power button being pushed, toggle the state.
  powerButtonState = !powerButtonState;
}

void checkTransmission(bool status, int retry) { // Recursively check if the request transmission was successful, if not retry transmission.
  LEDDebug(0, 255, 0, false); // Reset the retry LED to off.
  delay(250);

  if (status) { // Check if the request packet was sent.
    if (radio.isAckPayloadAvailable()) { // Check if the transmitter has sent an acknowledgement packet.
      radio.read(&received, sizeof(ackPacket)); // Read the acknowledgement packet.
    }

    if (received.isAck) { // Confirm acknowledgement of request reception.
      status = true;
      Serial.println("Request was receieved by the transmitter!");
    }
    else {
      status = false;
      Serial.println("Request was not received by the transmitter!");
    }
  }
  else if (!status && retry < 5) { // If the request packet was not received, then resend the request packet, for a maximum number of 5 retries.
    LEDDebug(255, 0, 0, true);
    Serial.println("Request failed, retrying...");
    Serial.println(retry);
    LEDDebug(0, 255, 0, true);
    status = radio.write(&request, sizeof(buttonPacket)); // Resend the request packet.
    LEDDebug(0, 255, 0, false);
    checkTransmission(status, retry + 1); // Recursively check if the retry was successful.
  }
  else if (retry == 5) { // If the maximum of retries has been reached, move on.
    Serial.println("Error: Request failed, check transmitter!");
  }
}