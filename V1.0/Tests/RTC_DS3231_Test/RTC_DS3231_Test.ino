/*
 * Programmer: Justin Lam
 * Date: 05/05/24
 * Description: DS3231 synchronisation test, to synchronise two RTC modules and check if synchronisation was
 *              successful.
*/

#include <RTClib.h>

String timeDate;
DateTime now;

RTC_DS3231 rtc; // Declare DS3231 RTC object.

void setup() {
  Serial.begin(9600);

  // Initialise the DS3231.
  while (!rtc.begin()) {
    Serial.println("Error: Failed to initialise DS3231!");
    Serial.println("Retrying...");
    delay(1000);
  }

  if (rtc.lostPower()) { // Reset the time and date if power loss occured.
    Serial.println("RTC has lost power, resetting the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set the time of the RTC to time of compilation.
  }

  Serial.println("DS3231 successfully initialised!");
  delay(500);
}

void loop() {
  now = rtc.now();

  if (now.second() % 5 == 0) { // Print the current time evary 5 seconds.
    timeDate = String(now.hour()) + ':' + String(now.minute()) + ':' + String(now.second()) + ' ' + String(now.day()) + '/' + String(now.month()) + '/' + String(now.year());
    Serial.println(timeDate);
    delay(800);
  }
}