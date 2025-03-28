#include "DHT.h"

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

#define SD_CARD_LOG_FILEPATH "IOT102.tsv"

// LM35
// #define LM35_PIN A0
#define DHT11_PIN 4

// #include "DHT.h"
DHT dht;

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

double dht11_get_temperature()
{
  delay(dht.getMinimumSamplingPeriod());
  double temperature = dht.getTemperature();
  return temperature;
}
// double lm35_get_temperature()
// {
// double v_out = analogRead(LM35_PIN) * (5.0 / 1023.0);
// // double celsius_temperature_value = (v_out * 500.0) / 1023.0;
// // double celsius_temperature_value = v_out * (500.0 / 1023.0);
// double celsius_temperature_value = v_out * 100.0;
// return celsius_temperature_value;
// }

RTC_DS3231 rtc_clock;
DateTime dt_obj;
// TODO sync datetime from phone

File sd_file_obj;

double TEMPERATURE_THRESHOLD = 31.5;
bool FORCE_FAN_ON            = false;

// clang-format off

const uint8_t _LED_8x8_DIGITS[10][8] = {
{
B00000000,
B00111100,
B01100110,
B01101110,
B01110110,
B01100110,
B01100110,
B00111100
},
// 0
{
B00000000,
B00110000,
B01110000,
B00110000,
B00110000,
B00110000,
B00110000,
B01111000
},
// 1
{
B00000000,
B01111000,
B11001100,
B00001100,
B00111000,
B01100000,
B11000000,
B11111100
},
// 2
{
B00000000,
B00111100,
B01100110,
B00000110,
B00011100,
B00000110,
B01100110,
B00111100,
},
// 3
{
B00000000,
B00001100,
B00011100,
B00101100,
B01001100,
B01111110,
B00001100,
B00001100,
},
// 4
{
B00000000,
B01111110,
B01100000,
B01111100,
B00000110,
B00000110,
B01100110,
B00111100,
},
// 5
{
B00000000,
B00111100,
B01100110,
B01100000,
B01111100,
B01100110,
B01100110,
B00111100,
},
// 6
{
B00000000,
B01111110,
B00000110,
B00001100,
B00011000,
B00110000,
B00110000,
B00110000,
},
// 7
{
B00000000,
B00111100,
B01100110,
B01100110,
B00111100,
B01100110,
B01100110,
B00111100,
},
// 8
{
B00000000,
B00111100,
B01100110,
B01100110,
B00111110,
B00000110,
B01100110,
B00111100,
}   // 9
};
const unsigned char _8x8_COLON[8] = {
B00000000,
B00011000,
B00011000,
B00000000,
B00000000,
B00011000,
B00011000,
B00000000
};  // :
const unsigned char _8x8_CHAR_C[8] = {
B00000000,
B00011110,
B00110011,
B00110000,
B00110000,
B00110000,
B00110011,
B00011110
};

// clang-format on

void display_number(int led_addr, unsigned char input_number)
{
  if (led_addr < 0)
  {
    return;
  }
  if (led_addr >= MAX7219_MAX_DEVICES)
  {
    return;
  }
  if (input_number > 9)
  {
    return;
  }
  lc.clearDisplay(led_addr);
  for (unsigned char row_idx = 0; row_idx < 8; row_idx++)
  {
    // lc.setRow(led_addr, row_idx, _LED_8x8_DIGITS[input_number][row_idx]);
    lc.setRow(led_addr, 7 - row_idx, _LED_8x8_DIGITS[input_number][row_idx]);
    // lc.setColumn(led_addr, 7-row_idx, _LED_8x8_DIGITS[input_number][row_idx]);
    // lc.setColumn(led_addr, row_idx, _LED_8x8_DIGITS[input_number][row_idx]);
    // lc.setRow(led_addr, 8-row_idx, _LED_8x8_DIGITS[input_number][row_idx]);
    for (unsigned char col_idx = 0; col_idx < 8; col_idx++)
    {
      bool led_state = (_LED_8x8_DIGITS[input_number][row_idx] >> col_idx & 1);
      lc.setLed(led_addr, 7 - row_idx, col_idx, led_state);
    }
  }
}

void clear_all_led_display()
{
  for (unsigned char led_addr = 0; led_addr < MAX7219_MAX_DEVICES; led_addr++)
  {
    lc.clearDisplay(led_addr);
  }
}

// {0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x00} // :
// {0x3C, 0x66, 0x60, 0x60, 0x60, 0x66, 0x3C, 0x00}  // C
void display_colon(int led_addr)
{
  if (led_addr < 0)
  {
    return;
  }
  if (led_addr >= MAX7219_MAX_DEVICES)
  {
    return;
  }
  lc.clearDisplay(led_addr);
  for (unsigned char row_idx = 0; row_idx < 8; row_idx++)
  {
    lc.setRow(led_addr, row_idx, _8x8_COLON[row_idx]);
  }
}

void display_C_character(int led_addr)
{
  if (led_addr < 0)
  {
    return;
  }
  if (led_addr >= MAX7219_MAX_DEVICES)
  {
    return;
  }
  lc.clearDisplay(led_addr);
  for (unsigned char row_idx = 0; row_idx < 8; row_idx++)
  {
    // lc.setRow(led_addr, row_idx, _8x8_CHAR_C[row_idx]);

    for (unsigned char col_idx = 0; col_idx < 8; col_idx++)
    {
      bool led_state = (_8x8_CHAR_C[row_idx] >> col_idx & 1);
      lc.setLed(led_addr, 7 - row_idx, col_idx, led_state);
    }
  }
}

void led_matrix_put_dot_for_temperature_display()
{
  lc.setLed(1, 0, 0, true);
  //   lc.setLed(1, 1, 0, true);
  //   lc.setLed(2, 0, 7, true);
  //   lc.setLed(2, 1, 7, true);
  lc.setLed(3, 7, 7, true);
  lc.setLed(3, 7, 6, true);
  lc.setLed(3, 6, 7, true);
}

void control_fan(double temp_c)
{
  if (FORCE_FAN_ON)
  {
    analogWrite(L9110_B_1A, 255);
  }
  else
  {
    if (temp_c >= TEMPERATURE_THRESHOLD)
    {
      analogWrite(L9110_B_1A, 255);
    }
    else
    {
      analogWrite(L9110_B_1A, 0);
    }
  }
}

void display_temperature_on_led_matrix(double temp_c)
{
  clear_all_led_display();
  unsigned char digit_1, digit_2;
  digit_1 = digit_2 = (unsigned char)(temp_c);
  digit_1           = ((digit_1 % 100) / 10) % 10;
  display_number(0, digit_1);
  digit_2 = digit_2 % 10;
  display_number(1, digit_2);
  unsigned char first_decimal_digit = (unsigned char)((int)(temp_c * 10) % 10);
  display_number(2, first_decimal_digit);
  display_C_character(3);
  led_matrix_put_dot_for_temperature_display();
}

void sd_card_test_read_ascii_art()
{
  // reading file from SD card
  sd_file_obj = SD.open("test.txt");
  if (sd_file_obj)
  {
    // Serial.println("test.txt:");
    // read from the file until there's nothing else in it:
    Serial.println(F("content of test.txt file from SD card"));
    while (sd_file_obj.available())
    {
      Serial.write(sd_file_obj.read());
    }

    // close the file:
    sd_file_obj.close();
  }
  else
  {
    // if the file didn't open, print an error
    Serial.println(F("failed to open test.txt"));
  }
}

void write_log_to_sd_card(double temp_c, uint32_t unix_ts)
{
  // write data to SD card
  sd_file_obj = SD.open(SD_CARD_LOG_FILEPATH, FILE_WRITE);
  if (sd_file_obj)
  {
    // sprintf(       //
    //     log_line,  //
    //     "%lu\t%.1f",  //
    //     unix_ts,   //
    //     temp_c     //
    // );
    char temp_str[10];
    dtostrf(temp_c, 4, 1, temp_str);

    char log_line[40];

    sprintf(        //
        log_line,   //
        "%lu\t%s",  //
        unix_ts,    //
        temp_str    //
    );

    // sprintf(          //
    //     log_line,     //
    //     "%lu\t%.1f",  //
    //     unix_ts,      //
    //     temp_c        //
    // );
    Serial.println(F("log line"));
    Serial.print(log_line);
    Serial.println(F("|"));
    sd_file_obj.println(log_line);
    sd_file_obj.close();
    Serial.println(F("log written to SD card"));
  }
}

#define MAX_LINES 20
#define MAX_LINE_LENGTH 64  // Adjust this based on expected line length

void sendLast20Lines() {
    File sd_file_obj = SD.open(SD_CARD_LOG_FILEPATH, FILE_READ);
    
    if (!sd_file_obj) {
        return; // File not found or error opening
    }

    BTSerial.write((unsigned char)0);

    // Array to store last 20 lines
    String lastLines[MAX_LINES];
    int lineCount = 0;

    // Read file line by line
    String line = "";
    while (sd_file_obj.available()) {
        char c = sd_file_obj.read();
        if (c == '\n') {
            if (line.length() > 0) {
                if (lineCount < MAX_LINES) {
                    lastLines[lineCount] = line;
                } else {
                    // Shift lines up and add new one at the end
                    for (int i = 1; i < MAX_LINES; i++) {
                        lastLines[i - 1] = lastLines[i];
                    }
                    lastLines[MAX_LINES - 1] = line;
                }
                lineCount++;
            }
            line = "";
        } else {
            line += c;
        }
    }
    sd_file_obj.close();

    // Send last 20 lines over Bluetooth
    for (int i = 0; i < min(lineCount, MAX_LINES); i++) {
        BTSerial.println(lastLines[i]);
    }
}


#define BT_CMD_CODE_SET_THRESHOLD 1
#define BT_CMD_CODE_ENABLE_FAN    2
#define BT_CMD_CODE_DISABLE_FAN   3
#define BT_CMD_CODE_DOWNLOAD_DATA 4
#define SD_CARD_READ_BUFFER_SIZE  16

void handle_bluetooth_communication()
{
  int cmd_code = BTSerial.read();
  Serial.print("handle_bluetooth_communication - cmd_code: ");
  Serial.println(cmd_code);
  if (cmd_code == BT_CMD_CODE_SET_THRESHOLD)
  {
    byte floatBytes[4];
    // Read 4 bytes into the array
    for (int i = 0; i < 4; i++) {
        floatBytes[i] = BTSerial.read();
        Serial.println(floatBytes[i]);
    }

    // Convert bytes to float (Little Endian)
    float receivedValue;
    memcpy(&receivedValue, floatBytes, 4);

    Serial.print("New temperature theshold: ");
    Serial.println(receivedValue);
    TEMPERATURE_THRESHOLD = receivedValue;
    BTSerial.write((unsigned char)0);
  }
  else if (cmd_code == BT_CMD_CODE_ENABLE_FAN)
  {
    FORCE_FAN_ON = true;
    analogWrite(L9110_B_1A, 255);
    BTSerial.write((unsigned char)0);
  }
  else if (cmd_code == BT_CMD_CODE_DISABLE_FAN)
  {
    FORCE_FAN_ON = false;
    analogWrite(L9110_B_1A, 0);
    BTSerial.write((unsigned char)0);
  }
  else if (cmd_code == BT_CMD_CODE_DOWNLOAD_DATA)
  {
    if (!SD_OK)
    {
      BTSerial.write((unsigned char)1);
    }
    else
    {
      BTSerial.write((unsigned char)0);
    }
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(HC05_TX, INPUT);
  pinMode(HC05_RX, OUTPUT);
  BTSerial.begin(9600);

  pinMode(L9110_B_1A, OUTPUT);
  // analogWrite(L9110_B_1A, LOW);
  analogWrite(L9110_B_1A, 255);
  dht.setup(DHT11_PIN);

  while (!Serial) { }

  // Initialize LED matrix
  int device_count = lc.getDeviceCount();
  for (int device_idx = 0; device_idx < device_count; device_idx++)
  {
    lc.shutdown(device_idx, false);  // Wake up display
    // TODO change intensity using potentiometer
    lc.setIntensity(device_idx, 0);  // Set brightness (0-15)
    lc.clearDisplay(device_idx);
  }

  if (SD.begin(SD_CARD_CS_PIN))
  {
    SD_OK = true;
    // Sd2Card card;
    // SdVolume volume;
    // SdFile root;
    Serial.println(F("SD OK"));
    sd_file_obj = SD.open(SD_CARD_LOG_FILEPATH, FILE_WRITE);
    if (sd_file_obj)
    {
      // sd_file_obj.println(log_line);
      sd_file_obj.write('\n');
      sd_file_obj.close();
      Serial.println(F("newline written to SD card"));
    }
    else
    {
      Serial.println(F("failed to open log file for initialize new session"));
    }
  }
  else
  {
    Serial.println(F("init SD failed!"));
    // Serial.println(F("SD"));
    // while (1) {}
  }

  // initialize RTC
  if (rtc_clock.begin())
  {
    DS3231_OK = true;

    if (rtc_clock.lostPower())
    {
      Serial.println("RTC lost power, let's set the time!");
      // When time needs to be set on a new device, or after a power loss, the
      // following line sets the RTC to the date & time this sketch was compiled
      rtc_clock.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
  }
  else
  {
    Serial.println(F("could not find RTC! STOP AT SETUP"));
    // Serial.println(F("RTC"));
    while (1)
    {
    }
  }

  // TODO test code
  // display_C_character(0);
  // display_colon(1);
  //   display_number(0, 9);
  //   display_number(1, 0);
  //   display_number(2, 1);
  //   display_number(3, 8);
  //   display_C_character(3);
  //   display_number(0, 9);
  //   display_number(1, 2);
  //   display_number(2, 2);
  //   display_number(3, 8);
  //   led_matrix_put_dot_for_temperature_display();
  //   while (1)
  //   {
  //   }
}

unsigned long last_log_time_ms = 0;
unsigned long log_interval_ms = 1000;

void loop()
{
  Serial.print(F("TEMPERATURE_THRESHOLD: "));
  Serial.println(TEMPERATURE_THRESHOLD);

  double _temp_start_ms = (double) millis();
  double temp_c = dht11_get_temperature();
  Serial.print(F("- DHT11: "));
  Serial.print(temp_c);
  Serial.println(F(" *C"));
  double _temp_time_ns = ((double)millis()) - _temp_start_ms;
  Serial.print("TEMP_TIME_MS: ");
  Serial.println(_temp_time_ns);

  double _led_start_ms = (double) millis();
  // Display time on LED matrix
  display_temperature_on_led_matrix(temp_c);
  double _led_time_ms = ((double) millis()) - _led_start_ms;
  Serial.print("_led_time_ms: ");
  Serial.println(_led_time_ms);

  control_fan(temp_c);

  if (DS3231_OK && SD_OK)
  {
    unsigned long current_time_ms = millis();
    if((current_time_ms - last_log_time_ms) >= log_interval_ms){
      last_log_time_ms = current_time_ms;

      Serial.println(F("- DS3231"));
      dt_obj           = rtc_clock.now();
      uint32_t unix_ts = dt_obj.unixtime();

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

      write_log_to_sd_card(temp_c, unix_ts);
      // sd_card_test_read_ascii_art();
    }

    double _total_time = ((double)millis()) - ((double)current_time_ms);
    Serial.print("SD_total_time: ");
    Serial.println(_total_time);
  }else{
    Serial.print("DS3231_OK: ");
    Serial.println(DS3231_OK);
    Serial.print("SD_OK: ");
    Serial.println(SD_OK);
  }

  if (BTSerial.available())
  {
    handle_bluetooth_communication();
  }
}
