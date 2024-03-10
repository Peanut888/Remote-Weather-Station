/*
 * Programmer: Justin Lam
 * Date: 01/03/24
 * Description: Transmit humidity and temeprature data via Lo-Ra radio.
*/

#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_SHTC3.h>

#define RFM95_IRQ 3
#define RFM95_RST 4
#define RFM95_CS 8
#define RFM95_FREQ 433.0

uint8_t buffLen = 0;
uint8_t dataLen = 0;
uint8_t buffer[RH_RF95_MAX_MESSAGE_LEN] = { 0 };
uint8_t data[RH_RF95_MAX_MESSAGE_LEN] = { 0 };
sensors_event_t humi, temp;

RH_RF95 rfm95(RFM95_CS, RFM95_IRQ);
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();

void setup() {
  Serial.begin(115200);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(20);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rfm95.init()) {
    Serial.println("Error: Failed to initialise RFM95!");
    Serial.println("Retrying...");
    delay(1000);
  }

  while (!rfm95.setFrequency(RFM95_FREQ)) {
    Serial.println("Error: Failed to set frequency to to 433HZ!");
    Serial.println("Retrying...");
    delay(1000);
  }

  Serial.print("Frequency set to ");
  Serial.print(RFM95_FREQ);
  Serial.println("Hz.");
  rfm95.setTxPower(23, false);
  Serial.println("RFM95 successfully intitialised!");

  while (!shtc3.begin()) {
    Serial.println("Error: Failed to initialise SHTC3!");
    Serial.println("Retrying...");
    delay(1000);
  }

  Serial.println("SHTC successfully initialised!");
  delay(500);
}

void loop() {
  shtc3.getEvent(&humi, &temp);
  dataLen = sprintf((char *)data, "%f %f", humi.relative_humidity, temp.temperature);
  //dataLen = strlen((char *)data);

  if (dataLen < RH_RF95_MAX_MESSAGE_LEN) {
    Serial.println((char *)data);
  }

  rfm95.send(data, dataLen);
  delay(20);
  rfm95.waitPacketSent();
  buffLen = sizeof(buffer);

  if (rfm95.waitAvailableTimeout(1000)) {
    if (rfm95.recv(buffer, &buffLen)) {
      Serial.print("Received: ");
      Serial.println((char *)buffer);
    }
    else {
      Serial.println("Failed to receive acknowledgement!");
    }
  }
  else {
    Serial.println("No acknowledgement!");
  }

  delay(1000);
}
