/*
 * Programmer: Justin Lam
 * Date: 02/03/24
 * Description: Transmit humidity and temeprature data via Lo-Ra radio.
*/

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_IRQ 3
#define RFM95_RST 4
#define RFM95_CS 8
#define RFM95_FREQ 433.0

void parse(int length, float *data, char *buffer);

uint8_t dataLen = 0;
float values[2] = {0.0};
char data[RH_RF95_MAX_MESSAGE_LEN] = { '\0' };
char acknowledge[] = "Data Ok!";

RH_RF95 rfm95(RFM95_CS, RFM95_IRQ);

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
  delay(500);
}

void loop() {
  if (rfm95.available()) {
    dataLen = sizeof(data);

    if (rfm95.recv((uint8_t *)data, &dataLen)) {
      RH_RF95::printBuffer("Received:", (uint8_t *)data, dataLen);
      Serial.println(data);
      parse(2, values, data);
      Serial.println(values[0]);
      Serial.println(values[1]);
      rfm95.send((uint8_t *)acknowledge, 9);
      rfm95.waitPacketSent();
    }
    else {
      Serial.println("Failed to receive data!");
    }
  }
}

void parse(int length, float *data, char *buffer)
{
  size_t len = strlen(buffer);
  char parseStr[30] = {'\0'}, tempStr[RH_RF95_MAX_MESSAGE_LEN] = {'\0'};

  strcpy(tempStr, buffer);
  strcpy(parseStr, strtok(tempStr, " "));
  data[0] = atof(parseStr);

  for (int i = 1; i < length - 1; i++)
  {
    strcpy(parseStr, strtok(NULL, " "));
    data[i] = atof(parseStr);
  }

  strcpy(parseStr, strtok(NULL, "\0"));
  data[length - 1] = atof(parseStr);
}