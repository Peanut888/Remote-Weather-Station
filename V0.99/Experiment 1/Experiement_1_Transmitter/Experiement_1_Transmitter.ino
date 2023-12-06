/*
Title: Weather Station Transmittter Experiment 1
Programmer: Justin Lam
Date: 10/02/23
Title: Experiment to test transmitting temperature, humidity, pressure and altitude.
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <dht.h>
#include <DFRobot_BMP3XX.h>

#define CALIBRATE_ABSOLUTE_DIFFERENCE

const int DHTPin = 24;
const byte address[6] = "00001";
int readDHT = 0;
int resultBMP388 = 0;
float tempDHT = 0.0;
float tempBMP = 0.0;
float samplingPeriod = 0.0;
float samplingFrequency = 0.0;

struct dataPackage {
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  float altitude = 0.0;
};

RF24 radio(7, 8);
dht DHT;
DFRobot_BMP388_I2C BMP388(&Wire, BMP388.eSDOVDD);
dataPackage data;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(120);
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();

  while (ERR_OK != (resultBMP388 = BMP388.begin())) {
    if (ERR_DATA_BUS == resultBMP388) {
      Serial.println("Data bus error!");
    }
    else if (ERR_IC_VERSION == resultBMP388) {
      Serial.println("Chip versions do not match!");
    }

    delay(3000);
  }

  Serial.println("BMP388 begin!");

  while (!BMP388.setSamplingMode(BMP388.eUltraPrecision)) {
    Serial.println("Set sampling mode failed, retrying...");
    delay(3000);
  }

  #ifdef CALIBRATE_ABSOLUTE_DIFFERENCE
  if (BMP388.calibratedAbsoluteDifference(50.0)) {
    Serial.println("Absolute difference base value set successfully!");
  }
  #endif

  samplingPeriod = BMP388.getSamplingPeriodUS();
  Serial.print("Sampling Period: ");
  Serial.print(samplingPeriod);
  Serial.println("us");
  samplingFrequency = 1000000 / samplingPeriod;
  Serial.print("Sampling Frequency: ");
  Serial.print(samplingFrequency);
  Serial.println("Hz");
  Serial.println();
  delay(1000);
}

void loop() {
  readDHT = DHT.read22(DHTPin);
  tempDHT = DHT.temperature;
  tempBMP = BMP388.readTempC();
  data.temperature = (tempDHT + tempBMP) / 2;
  data.humidity = DHT.humidity;
  data.pressure = BMP388.readPressPa();
  data.altitude = BMP388.readAltitudeM();
  // radio.write(&data, sizeof(dataPackage));
  for (int i = 0; i < 5; i++) {
    radio.write(&data, sizeof(dataPackage));
    delay(10);
  }

  Serial.print("Temperature: ");
  Serial.print(data.temperature);
  Serial.print("C | Humidity: ");
  Serial.print(data.humidity);
  Serial.print("% | Pressure: ");
  Serial.print(data.pressure);
  Serial.print("Pa | Altitude: ");
  Serial.print(data.altitude);
  Serial.println("m");
  delay(100);
}
