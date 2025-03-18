#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <Wire.h>
// #include <RTClib.h>
// #include <DHT.h>

// Define LED Matrix setup
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4 // 8x32 = 4 modules (each 8x8)
#define CLK_PIN 13
#define DATA_PIN 11
#define CS_PIN 10

// Initialize LED matrix
MD_Parola matrix = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

// Initialize RTC
// RTC_DS3231 rtc;

// Define DHT sensor
// #define DHTPIN 2
// #define DHTTYPE DHT11  // Change to DHT22 if using DHT22
// DHT dht(DHTPIN, DHTTYPE);

void setup()
{
  Serial.begin(9600);

  // Initialize LED matrix
  matrix.begin();
  matrix.setIntensity(5); // Adjust brightness (0-15)
  matrix.displayClear();

  // Initialize RTC
  // if (!rtc.begin()) {
  //   Serial.println("Couldn't find RTC");
  //   while (1);
  // }

  // Initialize DHT sensor
  // dht.begin();
}

void loop()
{
  // Get time from RTC
  // DateTime now = rtc.now();
  char timeStr[10];
  // sprintf(timeStr, "%02d:%02d", now.hour(), now.minute());
  sprintf(timeStr, "%02d:%02d", 9, 50);

  // Get temperature from DHT sensor
  // float temperature = dht.readTemperature(); // Celsius
  char tempStr[10];
  tempStr[0] = '5';
  tempStr[1] = '0';
  tempStr[0] = ' ';
  tempStr[0] = 'c';
  tempStr[0] = '\0';
  // if (!isnan(temperature)) {
  //   sprintf(tempStr, "%.1fC", temperature);
  // } else {
  //   sprintf(tempStr, "N/A");
  // }

  // Display time on LED matrix
  matrix.displayText(timeStr, PA_CENTER, 50, 1000, PA_SCROLL_LEFT);
  while (!matrix.displayAnimate())
    ; // Wait for scrolling to finish

  // Display temperature on LED matrix
  matrix.displayText(tempStr, PA_CENTER, 50, 1000, PA_SCROLL_LEFT);
  while (!matrix.displayAnimate())
    ; // Wait for scrolling to finish

  delay(1000); // Delay before repeating
}
