#include "DHT.h"

#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#include <LedControl.h>

// AnalogRTCLib@1.1.0
#include <RTClib.h>

// SD card
#define SD_CARD_CS_PIN 10
// SD:MISO -> R3:12
// SD:MOSI -> R3:11
// SD:SCK -> R3:13
// SD:CS -> R3:10

#define SD_CARD_LOG_FILEPATH "IOT102.tsv"
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

unsigned char MAX7219_PREVIOUS_BUFFER[32] = {0};
unsigned char MAX7219_CURRENT_BUFFER[32] = {0};

void set_led(int addr, int row, int col, bool state){
  int x = (addr * 8) + col;
  int y = row;
  if((x < 0) || (x >= 32)){
    Serial.print(F("set_led invalid x = "));
    Serial.println(x);
    return;
  }

  if((y < 0) || (y >= 8)){
    Serial.print(F("set_led invalid y = "));
    Serial.println(y);
    return;
  }

  if(state){
    MAX7219_CURRENT_BUFFER[x] |= (1 << y); // set bit
  }else{
    MAX7219_CURRENT_BUFFER[x] &= ~(1 << y); // clear bit
  }
}

void clear_display(int addr){
  if((addr < 0) || (addr >=4)){
    return;
  }

  int x_start = addr * 8;
  int x_end = (addr + 1) * 8;
  for(int x = x_start; x < x_end; x++){
    MAX7219_CURRENT_BUFFER[x] = 0;
  }
}

void set_row(int addr, int row, unsigned char value){
  if((addr < 0) || (addr >=4)){
    return;
  }

  int x_start = (addr * 8);
  for(int col = 0; col < 8; col++){
    bool state = (value >> col) & 1;
    int x = x_start + col;

    if(state){
      MAX7219_CURRENT_BUFFER[x] |= (1 << row); // set bit
    }else{
      MAX7219_CURRENT_BUFFER[x] &= ~(1 << row); // clear bit
    }
  }
}

void update_display(){
  for(int x = 0; x < 32; x++){
    if(MAX7219_CURRENT_BUFFER[x] != MAX7219_PREVIOUS_BUFFER[x]){
      int addr = x / 8;
      int col = x % 8;
      for(int row = 0; row < 8; row++){
        bool previous_state = (MAX7219_PREVIOUS_BUFFER[x] >> row) & 1;
        bool current_state = (MAX7219_CURRENT_BUFFER[x] >> row) & 1;
        if(previous_state != current_state){
          lc.setLed(addr, row, col, current_state);
        }
      }
      MAX7219_PREVIOUS_BUFFER[x] = MAX7219_CURRENT_BUFFER[x];
    }
  }
}

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
  clear_display(led_addr);
  for (unsigned char row_idx = 0; row_idx < 8; row_idx++)
  {
    set_row(led_addr, 7 - row_idx, _LED_8x8_DIGITS[input_number][row_idx]);
    // for (unsigned char col_idx = 0; col_idx < 8; col_idx++)
    // {
    //   bool led_state = (_LED_8x8_DIGITS[input_number][row_idx] >> col_idx & 1);
    //   set_led(led_addr, 7 - row_idx, col_idx, led_state);
    // }
  }
}

void clear_all_led_display()
{
  for (unsigned char led_addr = 0; led_addr < MAX7219_MAX_DEVICES; led_addr++)
  {
    clear_display(led_addr);
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
  clear_display(led_addr);
  for (unsigned char row_idx = 0; row_idx < 8; row_idx++)
  {
    set_row(led_addr, row_idx, _8x8_COLON[row_idx]);
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
  clear_display(led_addr);
  for (unsigned char row_idx = 0; row_idx < 8; row_idx++)
  {
    for (unsigned char col_idx = 0; col_idx < 8; col_idx++)
    {
      bool led_state = (_8x8_CHAR_C[row_idx] >> col_idx & 1);
      set_led(led_addr, 7 - row_idx, col_idx, led_state);
    }
  }
}

void led_matrix_put_dot_for_temperature_display()
{
  set_led(1, 0, 0, true);
  set_led(3, 7, 7, true);
  set_led(3, 7, 6, true);
  set_led(3, 6, 7, true);
}

double CURRENT_TEMPERATURE = 0.0;

void control_fan()
{
  if (FORCE_FAN_ON)
  {
    analogWrite(L9110_B_1A, 255);
  }
  else
  {
    if (CURRENT_TEMPERATURE >= TEMPERATURE_THRESHOLD)
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
  update_display();
}

void sd_card_test_read_ascii_art()
{
  // reading file from SD card
  sd_file_obj = SD.open("test.txt");
  if (sd_file_obj)
  {
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

#define BLUETOOTH_MAX_LINES_TO_SEND 60
#define BLUETOOTH_MAX_FILE_SIZE 4096

// void transfer_file_over_bluetooth_last_x_lines(){
//   // encode and send file size
//   uint32_t file_size = sd_file_obj.size();
//   Serial.print(F("FILE_SIZE: "));
//   Serial.println(file_size);
//   BTSerial.write(file_size & 0xff);
//   BTSerial.write((file_size >> 8) & 0xff);
//   BTSerial.write((file_size >> 16) & 0xff);
//   BTSerial.write((file_size >> 24) & 0xff);

//   int value;
//   uint32_t count = 0;
//   Serial.println(F("FILE_CONTENT\n"));

//   uint32_t line_seek[BLUETOOTH_MAX_LINES_TO_SEND];
//   int line_count_in_buffer = 0;

//   while (sd_file_obj.available()){
//     if(count >= file_size){
//       break;
//     }

//     value = sd_file_obj.read();
//     if(value < 0){
//       break;
//     }

//     count++;

//     if(value == '\n'){
//       if(line_count_in_buffer < BLUETOOTH_MAX_LINES_TO_SEND){
//         line_seek[line_count_in_buffer] = count;
//         line_count_in_buffer++;
//       }else{
//         // shift data up to add new one at the end
//         for(int i = 1; i < BLUETOOTH_MAX_LINES_TO_SEND; i++){
//           line_seek[i - 1] = line_seek[i];
//         }
//         line_seek[BLUETOOTH_MAX_LINES_TO_SEND-1] = count;
//       }
//     }
//   }

//   Serial.println(F("\n\nEOF"));
//   Serial.print(count);
//   Serial.print(F("/"));
//   Serial.print(file_size);
//   Serial.print(F("\n"));
//   sd_file_obj.close();
// }

void transfer_file_over_bluetooth(){
  // encode and send file size
  uint32_t file_size = sd_file_obj.size();
  Serial.print(F("FILE_SIZE: "));
  Serial.println(file_size);
  BTSerial.write(file_size & 0xff);
  BTSerial.write((file_size >> 8) & 0xff);
  BTSerial.write((file_size >> 16) & 0xff);
  BTSerial.write((file_size >> 24) & 0xff);

  int value;
  uint32_t count = 0;
  Serial.println(F("FILE_CONTENT\n"));
  while (sd_file_obj.available()){
    if(count >= file_size){
      break;
    }

    value = sd_file_obj.read();
    if(value < 0){
      break;
    }

    count++;

    Serial.write(value);
    BTSerial.write(value);
  }

  Serial.println(F("\n\nEOF"));
  Serial.print(count);
  Serial.print(F("/"));
  Serial.print(file_size);
  Serial.print(F("\n"));
  sd_file_obj.close();
}


#define BT_CMD_CODE_SET_THRESHOLD 1
#define BT_CMD_CODE_ENABLE_FAN    2
#define BT_CMD_CODE_DISABLE_FAN   3
#define BT_CMD_CODE_DOWNLOAD_DATA 4
#define BT_CMD_CODE_GET_TEMPERATURE 5
#define BT_CMD_CODE_GET_FORCE_FAN_ON_STATUS 5
#define SD_CARD_READ_BUFFER_SIZE  16

unsigned char double_buffer[4];

void handle_bluetooth_communication()
{
  int cmd_code = BTSerial.read();
  Serial.print(F("handle_bluetooth_communication - cmd_code: "));
  Serial.println(cmd_code);
  if (cmd_code == BT_CMD_CODE_SET_THRESHOLD)
  {
    Serial.println(F("BT_CMD_CODE_SET_THRESHOLD"));
    // Read 4 bytes into the array
    for (int i = 0; i < 4; i++) {
        double_buffer[i] = BTSerial.read();
        Serial.println(double_buffer[i]);
    }

    // Convert bytes to float (Little Endian)
    float receivedValue;
    memcpy(&receivedValue, double_buffer, 4);

    Serial.print(F("New temperature theshold: "));
    Serial.println(receivedValue);
    TEMPERATURE_THRESHOLD = receivedValue;
    BTSerial.write((unsigned char)0);
  }
  else if (cmd_code == BT_CMD_CODE_ENABLE_FAN)
  {
    Serial.println(F("BT_CMD_CODE_ENABLE_FAN"));
    FORCE_FAN_ON = true;
    control_fan();
    BTSerial.write((unsigned char)0);
  }
  else if (cmd_code == BT_CMD_CODE_DISABLE_FAN)
  {
    Serial.println(F("BT_CMD_CODE_DISABLE_FAN"));
    FORCE_FAN_ON = false;
    control_fan();
    BTSerial.write((unsigned char)0);
  }
  else if (cmd_code == BT_CMD_CODE_DOWNLOAD_DATA)
  {
    Serial.println(F("BT_CMD_CODE_DOWNLOAD_DATA"));
    if (!SD_OK)
    {
      BTSerial.write((unsigned char)1);
    }
    else
    {
      sd_file_obj = SD.open(SD_CARD_LOG_FILEPATH, FILE_READ);
      if (!sd_file_obj)
      {
        Serial.print(F("SD card failed to open file "));
        Serial.println(SD_CARD_LOG_FILEPATH);
        BTSerial.write((unsigned char)1);
      }else{
        BTSerial.write((unsigned char)0);
        transfer_file_over_bluetooth();
      }
    }
  }else if (cmd_code == BT_CMD_CODE_GET_TEMPERATURE){
    Serial.println(F("BT_CMD_CODE_GET_TEMPERATURE"));
    memcpy(double_buffer, &CURRENT_TEMPERATURE, 4);
    BTSerial.write(double_buffer[0]);
    BTSerial.write(double_buffer[1]);
    BTSerial.write(double_buffer[2]);
    BTSerial.write(double_buffer[3]);
  }else if (cmd_code == BT_CMD_CODE_GET_FORCE_FAN_ON_STATUS){
    Serial.println(F("BT_CMD_CODE_GET_FORCE_FAN_ON_STATUS"));
    if(FORCE_FAN_ON){
      BTSerial.write((unsigned char)1);
    }else{
      BTSerial.write((unsigned char)0);
    }
  }
  else{
    Serial.print(F("unknown command code: "));
    Serial.println(cmd_code);
    // BTSerial.write((unsigned char)1);
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println(F("setup()"));
  pinMode(HC05_TX, INPUT);
  pinMode(HC05_RX, OUTPUT);
  BTSerial.begin(9600);

  pinMode(L9110_B_1A, OUTPUT);
  analogWrite(L9110_B_1A, 255);
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
    bool log_file_exist = SD.exists(SD_CARD_LOG_FILEPATH);
    Serial.println(F("log_file_exist:"));
    Serial.println(log_file_exist);
    if(log_file_exist){
      Serial.println(F("opening log file"));
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
  }
  else
  {
    Serial.println(F("init SD failed!"));
    // Serial.println(F("SD"));
    // while (1) {}
  }

  // initialize RTC
  Serial.println(F("initializing RTC"));
  if (rtc_clock.begin())
  {
    Serial.println(F("DS3231_OK"));
    DS3231_OK = true;

    if (rtc_clock.lostPower())
    {
      Serial.println(F("RTC lost power, let's set the time!"));
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
  // display_number(0, 9);
  // display_number(1, 2);
  // display_number(2, 2);
  // display_number(3, 8);
  // led_matrix_put_dot_for_temperature_display();

  // update_display();
  // while (1)
  // {
  // }
}

unsigned long last_log_time_ms = 0;
unsigned long log_interval_ms = 1000;

void loop()
{
  // Serial.println(F("loop()"));
  Serial.print(F("CURRENT_TEMPERATURE_THRESHOLD: "));
  Serial.println(TEMPERATURE_THRESHOLD);

  double _temp_start_ms = (double) millis();
  CURRENT_TEMPERATURE = dht11_get_temperature();
  Serial.print(F("- DHT11: "));
  Serial.print(CURRENT_TEMPERATURE);
  Serial.println(F(" *C"));
  double _temp_time_ns = ((double)millis()) - _temp_start_ms;
  Serial.print(F("TEMP_TIME_MS: "));
  Serial.println(_temp_time_ns);

  double _led_start_ms = (double) millis();
  // Display time on LED matrix
  display_temperature_on_led_matrix(CURRENT_TEMPERATURE);
  double _led_time_ms = ((double) millis()) - _led_start_ms;
  Serial.print(F("_led_time_ms: "));
  Serial.println(_led_time_ms);

  control_fan();

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

      write_log_to_sd_card(CURRENT_TEMPERATURE, unix_ts);
      // sd_card_test_read_ascii_art();
    }

    double _total_time = ((double)millis()) - ((double)current_time_ms);
    Serial.print(F("SD_total_time: "));
    Serial.println(_total_time);
  }else{
    Serial.print(F("DS3231_OK: "));
    Serial.println(DS3231_OK);
    Serial.print(F("SD_OK: "));
    Serial.println(SD_OK);
  }

  if (BTSerial.available())
  {
    handle_bluetooth_communication();
  }
}
