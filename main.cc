/******************************************************************
  DHT Temperature & Humidity Sensor library for Arduino.

  Features:
  - Support for DHT11 and DHT22/AM2302/RHT03
  - Auto detect sensor model
  - Very low memory footprint
  - Very small code

  http://www.github.com/markruys/arduino-DHT

  Written by Mark Ruys, mark@paracas.nl.

  BSD license, check license.txt for more information.
  All text above must be included in any redistribution.

  Datasheets:
  - http://www.micro4you.com/files/sensor/DHT11.pdf
  - http://www.adafruit.com/datasheets/DHT22.pdf
  - http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Weather/RHT03.pdf
  - http://meteobox.tk/files/AM2302.pdf

  Changelog:
   2013-06-10: Initial version
   2013-06-12: Refactored code
   2013-07-01: Add a resetTimer method
 ******************************************************************/

#ifndef dht_h
#define dht_h

#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

class DHT
{
 public:
  typedef enum
  {
    AUTO_DETECT,
    DHT11,
    DHT22,
    AM2302,  // Packaged DHT22
    RHT03    // Equivalent to DHT22
  } DHT_MODEL_t;

  typedef enum
  {
    ERROR_NONE = 0,
    ERROR_TIMEOUT,
    ERROR_CHECKSUM
  } DHT_ERROR_t;

  void setup(uint8_t pin, DHT_MODEL_t model = AUTO_DETECT);
  void resetTimer();

  float getTemperature();
  float getHumidity();

  DHT_ERROR_t getStatus() { return error; };
  const char* getStatusString();

  DHT_MODEL_t getModel() { return model; }

  int getMinimumSamplingPeriod() { return model == DHT11 ? 1000 : 2000; }

  int8_t getNumberOfDecimalsTemperature() { return model == DHT11 ? 0 : 1; };
  int8_t getLowerBoundTemperature() { return model == DHT11 ? 0 : -40; };
  int8_t getUpperBoundTemperature() { return model == DHT11 ? 50 : 125; };

  int8_t getNumberOfDecimalsHumidity() { return 0; };
  int8_t getLowerBoundHumidity() { return model == DHT11 ? 20 : 0; };
  int8_t getUpperBoundHumidity() { return model == DHT11 ? 90 : 100; };

  static float toFahrenheit(float fromCelcius) { return 1.8 * fromCelcius + 32.0; };
  static float toCelsius(float fromFahrenheit) { return (fromFahrenheit - 32.0) / 1.8; };

 protected:
  void readSensor();

  float temperature;
  float humidity;

  uint8_t pin;

 private:
  DHT_MODEL_t model;
  DHT_ERROR_t error;
  unsigned long lastReadTime;
};

#endif /*dht_h*/

/******************************************************************
  DHT Temperature & Humidity Sensor library for Arduino.

  Features:
  - Support for DHT11 and DHT22/AM2302/RHT03
  - Auto detect sensor model
  - Very low memory footprint
  - Very small code

  http://www.github.com/markruys/arduino-DHT

  Written by Mark Ruys, mark@paracas.nl.

  BSD license, check license.txt for more information.
  All text above must be included in any redistribution.

  Datasheets:
  - http://www.micro4you.com/files/sensor/DHT11.pdf
  - http://www.adafruit.com/datasheets/DHT22.pdf
  - http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Weather/RHT03.pdf
  - http://meteobox.tk/files/AM2302.pdf

  Changelog:
   2013-06-10: Initial version
   2013-06-12: Refactored code
   2013-07-01: Add a resetTimer method
 ******************************************************************/

//  #include "DHT.h"

void DHT::setup(uint8_t pin, DHT_MODEL_t model)
{
  DHT::pin   = pin;
  DHT::model = model;
  DHT::resetTimer();  // Make sure we do read the sensor in the next readSensor()

  if (model == AUTO_DETECT)
  {
    DHT::model = DHT22;
    readSensor();
    if (error == ERROR_TIMEOUT)
    {
      DHT::model = DHT11;
      // Warning: in case we auto detect a DHT11, you should wait at least 1000 msec
      // before your first read request. Otherwise you will get a time out error.
    }
  }
}

void DHT::resetTimer()
{
  DHT::lastReadTime = millis() - 3000;
}

float DHT::getHumidity()
{
  readSensor();
  return humidity;
}

float DHT::getTemperature()
{
  readSensor();
  return temperature;
}

#ifndef OPTIMIZE_SRAM_SIZE

const char* DHT::getStatusString()
{
  switch (error)
  {
    case DHT::ERROR_TIMEOUT:
      return "TIMEOUT";

    case DHT::ERROR_CHECKSUM:
      return "CHECKSUM";

    default:
      return "OK";
  }
}

#else

// At the expense of 26 bytes of extra PROGMEM, we save 11 bytes of
// SRAM by using the following method:

prog_char P_OK[] PROGMEM       = "OK";
prog_char P_TIMEOUT[] PROGMEM  = "TIMEOUT";
prog_char P_CHECKSUM[] PROGMEM = "CHECKSUM";

const char* DHT::getStatusString()
{
  prog_char* c;
  switch (error)
  {
    case DHT::ERROR_CHECKSUM:
      c = P_CHECKSUM;
      break;

    case DHT::ERROR_TIMEOUT:
      c = P_TIMEOUT;
      break;

    default:
      c = P_OK;
      break;
  }

  static char buffer[9];
  strcpy_P(buffer, c);

  return buffer;
}

#endif

void DHT::readSensor()
{
  // Make sure we don't poll the sensor too often
  // - Max sample rate DHT11 is 1 Hz   (duty cicle 1000 ms)
  // - Max sample rate DHT22 is 0.5 Hz (duty cicle 2000 ms)
  unsigned long startTime = millis();
  if ((unsigned long)(startTime - lastReadTime) < (model == DHT11 ? 999L : 1999L))
  {
    return;
  }
  lastReadTime = startTime;

  temperature = NAN;
  humidity    = NAN;

  // Request sample

  digitalWrite(pin, LOW);  // Send start signal
  pinMode(pin, OUTPUT);
  if (model == DHT11)
  {
    delay(18);
  }
  else
  {
    // This will fail for a DHT11 - that's how we can detect such a device
    delayMicroseconds(800);
  }

  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);  // Switch bus to receive data

  // We're going to read 83 edges:
  // - First a FALLING, RISING, and FALLING edge for the start bit
  // - Then 40 bits: RISING and then a FALLING edge per bit
  // To keep our code simple, we accept any HIGH or LOW reading if it's max 85 usecs long

  uint16_t rawHumidity    = 0;
  uint16_t rawTemperature = 0;
  uint16_t data           = 0;

  for (int8_t i = -3; i < 2 * 40; i++)
  {
    byte age;
    startTime = micros();

    do
    {
      age = (unsigned long)(micros() - startTime);
      if (age > 90)
      {
        error = ERROR_TIMEOUT;
        return;
      }
    } while (digitalRead(pin) == (i & 1) ? HIGH : LOW);

    if (i >= 0 && (i & 1))
    {
      // Now we are being fed our 40 bits
      data <<= 1;

      // A zero max 30 usecs, a one at least 68 usecs.
      if (age > 30)
      {
        data |= 1;  // we got a one
      }
    }

    switch (i)
    {
      case 31:
        rawHumidity = data;
        break;
      case 63:
        rawTemperature = data;
        data           = 0;
        break;
    }
  }

  // Verify checksum

  if ((byte)(((byte)rawHumidity) + (rawHumidity >> 8) + ((byte)rawTemperature) + (rawTemperature >> 8)) != data)
  {
    error = ERROR_CHECKSUM;
    return;
  }

  // Store readings

  if (model == DHT11)
  {
    humidity    = rawHumidity >> 8;
    temperature = rawTemperature >> 8;
  }
  else
  {
    humidity = rawHumidity * 0.1;

    if (rawTemperature & 0x8000)
    {
      rawTemperature = -(int16_t)(rawTemperature & 0x7FFF);
    }
    temperature = ((int16_t)rawTemperature) * 0.1;
  }

  error = ERROR_NONE;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

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
    // TODO
  }
  else if (cmd_code == BT_CMD_CODE_ENABLE_FAN)
  {
    FORCE_FAN_ON = true;
    
  analogWrite(L9110_B_1A, 255);
    // BTSerial.write((unsigned char)0);
  }
  else if (cmd_code == BT_CMD_CODE_DISABLE_FAN)
  {
    FORCE_FAN_ON = false;
    
  analogWrite(L9110_B_1A, 0);
    // BTSerial.write((unsigned char)0);
  }
  else if (cmd_code == BT_CMD_CODE_DOWNLOAD_DATA)
  {
    if (!SD_OK)
    {
      BTSerial.write((unsigned char)1);
    }
    else
    {
      sd_file_obj = SD.open(SD_CARD_LOG_FILEPATH, FILE_READ);
      if (sd_file_obj)
      {
        BTSerial.write((unsigned char)0);
        uint32_t file_size = sd_file_obj.size();
        BTSerial.write(file_size & 0xff);
        BTSerial.write((file_size >> 8) & 0xff);
        BTSerial.write((file_size >> 16) & 0xff);
        BTSerial.write((file_size >> 24) & 0xff);

        uint32_t bluetooth_total_sent_byte_count = 0;

        char read_buffer[SD_CARD_READ_BUFFER_SIZE];
        uint32_t total_read_count = 0;
        // reading file data from SD card
        while (1)
        {
          size_t read_count = sd_file_obj.readBytes(read_buffer, SD_CARD_READ_BUFFER_SIZE);
          if (read_count < 1)
          {
            break;
          }

          total_read_count += read_count;

          size_t total_sent_count  = 0;
          size_t sent_count        = 0;
          size_t remain_byte_count = read_count;

          // sending data over serial interface
          while (1)
          {
            Serial.print(F("BLUETOOTH_SENDING_DATA: "));
            Serial.print(F("FILE_SIZE: "));
            Serial.print(file_size);
            Serial.print(F(" SENT_COUNT: "));
            Serial.print(bluetooth_total_sent_byte_count);
            Serial.print(F("\n"));

            sent_count = BTSerial.write(read_buffer + total_sent_count, remain_byte_count);
            if (sent_count < 1)
            {
              break;
            }

            total_sent_count += sent_count;
            remain_byte_count -= sent_count;
            bluetooth_total_sent_byte_count += sent_count;

            if (remain_byte_count == 0)
            {
              break;
            }
          }

          if (sent_count < 1)
          {
            break;
          }
        }

        Serial.print(F("\n\nEND_BLUETOOTH_SENDING_DATA\n"));
      }
      else
      {
        Serial.println(F("failed to open file on SD card"));
        BTSerial.write((unsigned char)1);
      }
    }
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(HC05_TX, INPUT);
  pinMode(HC05_RX, OUTPUT);
  BTSerial.begin(9600);
  // BTSerial.begin(38400);

  pinMode(L9110_B_1A, OUTPUT);
  // analogWrite(L9110_B_1A, LOW);
  analogWrite(L9110_B_1A, 255);
  // pinMode(LM35_PIN, INPUT);
  dht.setup(DHT11_PIN);

  // while (!Serial) { }

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

void loop()
{
  Serial.print(F("TEMPERATURE_THRESHOLD: "));
  Serial.println(TEMPERATURE_THRESHOLD);

  double temp_c = dht11_get_temperature();
  Serial.print(F("- DHT11: "));
  Serial.print(temp_c);
  Serial.println(F(" *C"));

  control_fan(temp_c);
  // Display time on LED matrix
  display_temperature_on_led_matrix(temp_c);

  if (DS3231_OK && SD_OK)
  {
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
    sd_card_test_read_ascii_art();
  }

  if (BTSerial.available())
  {
    handle_bluetooth_communication();
  }

  delay(500);
}
