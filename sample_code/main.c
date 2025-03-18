#include <SPI.h>
#include <Wire.h>

// MD_Parola@3.7.3
// MD_MAX72XX@3.5.1
#include <MD_Parola.h>
#include <MD_MAX72xx.h>


// AnalogRTCLib@1.1.0
#include <RTClib.h>

// define LED Matrix setup
#define MAX7219_HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX7219_MAX_DEVICES 4  // 8x32 = 4 modules (each 8x8)
#define MAX7219_CLK_PIN  13
#define MAX7219_DATA_PIN 11
#define MAX7219_CS_PIN   10

// MAX7219:CLK -> R3:13
// MAX7219:DIN -> R3:11
// MAX7219:CS -> R3:10
// initialize LED matrix
MD_Parola matrix = MD_Parola(
    MAX7219_HARDWARE_TYPE,
    MAX7219_DATA_PIN,
    MAX7219_CLK_PIN,
    MAX7219_CS_PIN,
    MAX7219_MAX_DEVICES
);

// DS3231:SCL -> R3:SCL
// DS3231:SDA -> R3:SDA
RTC_DS3231 rtc_clock;

// TODO sync datetime from phone

void setup() {
  Serial.begin(9600);

  // initialize LED matrix
  matrix.begin();
  // matrix.setIntensity(2);  // Adjust brightness (0-15)
  matrix.setIntensity(1);  // Adjust brightness (0-15)
  matrix.displayClear();

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

  // TODO threading?
  // Display time on LED matrix
  matrix.displayClear();
  matrix.displayText(date_time_str, PA_CENTER, 50, 1000, PA_SCROLL_LEFT);
  while (!matrix.displayAnimate());  // Wait for scrolling to finish

  // delay(1000);
  delay(100);
  // char pad_str[3] = {
  //   ' ',
  //   ' ',
  //   '\0'
  // };

  // matrix.displayText(pad_str, PA_CENTER, 50, 1000, PA_SCROLL_LEFT);
  // while (!matrix.displayAnimate());  // Wait for scrolling to finish
}
