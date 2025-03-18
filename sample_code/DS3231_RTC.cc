#include <SPI.h>
#include <Wire.h>

// DS3231:SCL -> R3:SCL
// DS3231:SDA -> R3:SDA

// AnalogRTCLib@1.1.0
#include <RTClib.h>

RTC_DS3231 rtc_clock;

// TODO sync datetime from phone

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // initialize RTC
  if(!rtc_clock.begin()){
    Serial.println("could not find RTC");
    while(1);
  }
}

void loop() {
  DateTime now = rtc_clock.now();
  char date_time_str[20];
  sprintf(
    date_time_str,
    "%04d-%02d-%02d %02d:%02d:%02d",  // 4+1+2+1+2+1+2*3+2=19
    now.year(),
    now.month(),
    now.dayOfTheWeek(),
    now.hour(),
    now.minute(),
    now.second()
  );
  date_time_str[19] = '\0';
  Serial.println(date_time_str);
  delay(1000);
}
