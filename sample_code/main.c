#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#include <LedControl.h>

// AnalogRTCLib@1.1.0
#include <RTClib.h>

// SD card
#define SD_CARD_CS_PIN 10
// SD:MISO -> R3:11
// SD:MOSI -> R3:12
// SD:SCK -> R3:13
// SD:CS -> R3:10
// LM35
#define LM35_PIN       A0

// define LED Matrix setup

#define MAX7219_MAX_DEVICES 4  // 8x32 = 4 modules (each 8x8)
#define MAX7219_CLK_PIN     8
#define MAX7219_CS_PIN      7
#define MAX7219_DATA_PIN    6

// MAX7219:CLK -> R3:8
// MAX7219:CS -> R3:7
// MAX7219:DIN -> R3:6
// initialize LED matrix
LedControl lc = LedControl(MAX7219_DATA_PIN, MAX7219_CLK_PIN, MAX7219_CS_PIN, MAX7219_MAX_DEVICES);

// DS3231:SCL -> R3:SCL
// DS3231:SDA -> R3:SDA
bool DS3231_OK = false;
bool SD_OK     = false;

// HC-05
// HC-05:TX -> R3:2
// HC-05:RX -> R3:3
#include <SoftwareSerial.h>
#define HC05_TX 2
#define HC05_RX 3
SoftwareSerial BTSerial(HC05_TX, HC05_RX);

// L9110
// L9110:B-1A -> R3:5
// L9110:B-1B -> GND
// L9110:MOTOR_B:LEFT -> 12V_FAN_GND
// L9110:MOTOR_B:RIGHT -> 12V_FAN_VCC
// L9110:GND -> 12V_PSU:GND
// L9110:VCC -> 12V_PSU:VCC

// |=============================================|
// |                    L9110                    |
// | LEFT PIN | RIGHT PIN | LEFT PIN | RIGHT PIN |
// | MOTOR B              |  MOTOR A             |
// |=============================================|
// | B-1A | B-1B |    GND | VCC    | A-1A | A-1B |
// |=============================================|
#define L9110_B_1A 5

double lm35_get_temperature()
{
  double v_out = analogRead(LM35_PIN) * (5.0 / 1023.0);
  // double celsius_temperature_value = (v_out * 500.0) / 1023.0;
  // double celsius_temperature_value = v_out * (500.0 / 1023.0);
  double celsius_temperature_value = v_out * 100.0;
  return celsius_temperature_value;
}

RTC_DS3231 rtc_clock;
DateTime dt_obj;
// TODO sync datetime from phone

File sd_file_obj;

void setup()
{
  Serial.begin(9600);
  BTSerial.begin(9600);

  pinMode(L9110_B_1A, OUTPUT);
  // analogWrite(L9110_B_1A, LOW);
  analogWrite(L9110_B_1A, 255);
  pinMode(LM35_PIN, INPUT);

  // while (!Serial) { }

  // Initialize LED matrix
  int device_count = lc.getDeviceCount();
  for (int device_idx = 0; device_idx < device_count; device_idx++)
  {
    lc.shutdown(device_idx, false);  // Wake up display
    lc.setIntensity(device_idx, 0);  // Set brightness (0-15)
    lc.clearDisplay(device_idx);
  }

  if (SD.begin(SD_CARD_CS_PIN)) {
    SD_OK = true;
    // Sd2Card card;
    // SdVolume volume;
    // SdFile root;
    Serial.println(F("SD OK"));

  }
  else {
    Serial.println(F("init SD failed!"));
    // Serial.println(F("SD"));
    // while (1) {}
  }

  // initialize RTC
  if (rtc_clock.begin()) { DS3231_OK = true; }
  else {
    Serial.println(F("could not find RTC"));
    // Serial.println(F("RTC"));
    while (1) {}
  }
}

void loop()
{
  if (DS3231_OK){
    Serial.println(F("- DS3231"));
    dt_obj = rtc_clock.now();

    char date_time_str[20];
    sprintf(                              //
        date_time_str,                    //
        "%04d-%02d-%02d %02d:%02d:%02d",  // 4+1+2+1+2+1+2*3+2=19
        dt_obj.year(),                    //
        dt_obj.month(),                   //
        dt_obj.dayOfTheWeek(),            //
        dt_obj.hour(),                    //
        dt_obj.minute(),                  //
        dt_obj.second()                   //
    );

    date_time_str[19] = '\0';
    Serial.println(date_time_str);
  }

  double temp_c = lm35_get_temperature();
  Serial.print(F("- LM35: "));
  Serial.print(temp_c);
  Serial.println(F(" *C"));

  // TODO threading?
  // Display time on LED matrix

  if (SD_OK){
    // reading file from SD card
    sd_file_obj = SD.open("test.txt");
    if (sd_file_obj){
      // Serial.println("test.txt:");
      // read from the file until there's nothing else in it:
      Serial.println(F("content of test.txt file from SD card"));
      while (sd_file_obj.available()){
        Serial.write(sd_file_obj.read());
      }

      // close the file:
      sd_file_obj.close();
    }else{
      // if the file didn't open, print an error
      Serial.println(F("failed to open test.txt"));
    }
  }

  delay(1000);
  analogWrite(L9110_B_1A, LOW);

  for(int led_addr = 0; led_addr < 4; led_addr++){
    lc.clearDisplay(led_addr);
    for(int row_idx = 0; row_idx < 8; row_idx++){
      for(int col_idx = 0; col_idx < 8; col_idx++){
        lc.setLed(led_addr, row_idx, col_idx, true);
        // delay(100);
        // delay(50);
        // delay(10);
        delay(10);
      }
      lc.clearDisplay(led_addr);
    }
    lc.clearDisplay(led_addr);
  }
  // delay(3000);
  analogWrite(L9110_B_1A, 255);
  delay(3000);
}
