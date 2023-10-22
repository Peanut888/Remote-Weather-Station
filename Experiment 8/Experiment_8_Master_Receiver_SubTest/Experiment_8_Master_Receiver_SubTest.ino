/*
Title: Weather Station Receiver Experiment 8
Programmer: Justin Lam
Date: 28/02/23
Description: Data logging experiment, master arduino that receives the data and sends in via I2C, with 2 buttons to control the OLED and transmission.
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

const int getDataButtonPin = 2;
const int blueLedPin = 3; // RGB blue pin.
const int greenLedPin = 5; // RGB green pin.
const int redLedPin = 6; // RGB red pin.
const int CEPin = 7; // nRF24 chip enable pin.
const int CSNPin = 8; // nRF24 chip select pin.
const int OLEDPowerButtonPin = 19;
const byte address[2][6] = {"00001", "00002"}; // Define the address through which the radios will use to communicate.
int dataButtonState = 0;
int numRetry = 0;
bool isSuccess = false;
bool ackSuccess = false; // Acknowledgement status.
bool powerButtonState = true;

struct dataPacket {
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  float altitude = 0.0;
  float rpm = 0.0;
  int indexVOC = 0;
  int indexUV = 0;
  int rain = 1;
};

struct buttonPacket {
  bool buttonState = false;
};

struct ackPacket {
  bool isAck = false;
};

RF24 radio(CEPin, CSNPin); // Declare and initialise the radio where CE and CSN are digital pins 7 and 8 respectively.
Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Declare and initialise the SSD1306 setting I2C address 0x3D.
dataPacket data; // Declare the data struct named "data".
ackPacket received; // Declare and intitialise acknowledge struct named "received".
buttonPacket transmit;

void setup() {
  Serial.begin(9600); // Initialise serial communication.

  // Debug LED Initialisation
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  digitalWrite(redLedPin, HIGH);
  digitalWrite(greenLedPin, HIGH);
  digitalWrite(blueLedPin, HIGH);

  #ifndef ESP8266
  while (!Serial) {
    blinkLEDDebug(255, 0, 0, 500, BLINK_COUNT);
  }
  #endif

  // SSD1306 Initialisation
  delay(100);
  
  if (!OLED.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.print("Error: SSD1306 not responding!");
    blinkLEDDebug(255, 0, 0, 500, BLINK_COUNT);
    LEDDebug(255, 0, 0, true);
    while(1);
  }
  else {
    OLED.clearDisplay();
    displayText(1, WHITE, 15, 28, "Initialising...");
    delay(1500);
  }

  if (!radio.begin()) {
    Serial.println("Error: nRF24 radio not responding!");
    Serial.println("Please check wiring...");
    displayText(1, WHITE, 0, 28, "nRF24 not responding!");
    blinkLEDDebug(255, 0, 0, 500, BLINK_COUNT);
    LEDDebug(255, 0, 0, true);
    while(1);
  }
  else {
    radio.openWritingPipe(address[1]);
    radio.openReadingPipe(1, address[0]);
    radio.setPALevel(RF24_PA_LOW);
    radio.enableAckPayload();
    //radio.writeAckPayload(1, &received, sizeof(ackPacket));
    Serial.println("nRF24 Initialised!");
    displayText(1, WHITE, 0, 28, "nRF24 Initialised!");
    blinkLEDDebug(0, 255, 0, 200, BLINK_COUNT);
    delay(250);
  }

  pinMode(getDataButtonPin, INPUT);
  pinMode(OLEDPowerButtonPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(getDataButtonPin), getData, RISING);
  attachInterrupt(digitalPinToInterrupt(OLEDPowerButtonPin), pwrOLED, RISING);

  displayText(1, WHITE, 0, 26, "All Initialised!");
  delay(500);
}

void loop() {
  if (!powerButtonState) {
    OLED.clearDisplay();
    OLED.display();
  }
  else {
    displayData();
  }

  if (dataButtonState == 1) {
    transmit.buttonState = true;
    radio.stopListening();
    isSuccess = radio.write(&transmit, sizeof(buttonPacket));
    delay(50);
    dataButtonState = 0;
    numRetry = 0;
    radio.startListening();
    checkTransmission(isSuccess, numRetry);
    delay(50);
  }

  radio.startListening();
  delay(10);

  if (radio.available()) {
    radio.read(&data, sizeof(dataPacket));
    //LEDDebug(0, 0, 255, true);
    received.isAck = true;
    ackSuccess = radio.writeAckPayload(1, &received, sizeof(ackPacket));

    if (ackSuccess) {
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

      if (powerButtonState) {
        displayData();
        /*
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

        if (data.rain == 0) {
          OLED.println("Rain: Yes");
        }
        else {
          OLED.println("Rain: No");
        }  

        OLED.display();   
        */   
      }
      
      /*
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
      */
      
      if (data.rain == 0) {
        Serial.println("Rain: Yes");
        //OLED.println("Rain: Yes");
      }
      else {
        Serial.println("Rain: No");
        //OLED.println("Rain: No");
      }

      //OLED.display();
      delay(50);
    }
    else {
      Serial.println("Error: Payload acknowledgement failed!");
      blinkLEDDebug(255, 0, 0, 100, BLINK_COUNT);
    }

    received.isAck = false;
  }
  else {
    blinkLEDDebug(0, 0, 255, 100, 1);
  }
}

void LEDDebug(int red, int green, int blue, bool status) {
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

void blinkLEDDebug(int red, int green, int blue, int time, int count) {
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

void displayText(uint8_t size, uint16_t colour, uint16_t x, uint16_t y, String text) {
  OLED.clearDisplay();
  OLED.setTextColor(colour);
  OLED.setTextSize(size);
  OLED.setCursor(x, y);
  OLED.println(text);
  OLED.display();
}

void displayData() {
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

  if (data.rain == 0) {
    OLED.println("Yes");
  }
  else {
    OLED.println("No");
  }

  OLED.display();
}

void getData() {
  dataButtonState = 1;
}

void pwrOLED() {
  powerButtonState = !powerButtonState; 
}

void checkTransmission(bool status, int retry) {
  LEDDebug(0, 255, 0, false);
  delay(250);

  if (status) {
    if (radio.isAckPayloadAvailable()) {
      radio.read(&received, sizeof(ackPacket));
    }

    if (received.isAck) {
      status = true;
      Serial.println("Request was receieved by the transmitter!");
    }
    else {
      status = false;
      Serial.println("Request was not received by the transmitter!");
    }
  }
  else if (!status && retry < 5) {
    LEDDebug(255, 0, 0, true);
    Serial.println("Request failed, retrying...");
    Serial.println(retry);
    LEDDebug(0, 255, 0, true);
    status = radio.write(&transmit, sizeof(buttonPacket));
    LEDDebug(0, 255, 0, false);
    checkTransmission(status, retry + 1);
  }
  else if (retry == 5) {
    Serial.println("Error: Request failed, check transmitter!");
  }
}
