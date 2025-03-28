#include <SPI.h>
#include <SD.h>
#include <Wire.h>
// HC-05
// HC-05:TX -> R3:2
// HC-05:RX -> R3:3
#include <SoftwareSerial.h>
#define HC05_TX 2
#define HC05_RX 3
SoftwareSerial BTSerial(HC05_TX, HC05_RX);

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
    // TEMPERATURE_THRESHOLD = receivedValue;
    // TODO
    BTSerial.write((unsigned char)0);
  }
  else if (cmd_code == BT_CMD_CODE_ENABLE_FAN)
  {
    // FORCE_FAN_ON = true;

  // analogWrite(L9110_B_1A, 255);
    BTSerial.write((unsigned char)0);
  }
  else if (cmd_code == BT_CMD_CODE_DISABLE_FAN)
  {
    // FORCE_FAN_ON = false;

  // analogWrite(L9110_B_1A, 0);
    BTSerial.write((unsigned char)0);
    Serial.println("BT_CMD_CODE_DISABLE_FAN");
  }
  else if (cmd_code == BT_CMD_CODE_DOWNLOAD_DATA)
  {
    Serial.println("BT_CMD_CODE_DOWNLOAD_DATA");
    // TODO
    //   sd_file_obj = SD.open(SD_CARD_LOG_FILEPATH, FILE_READ);
    //   if (sd_file_obj)
    //   {
    //     BTSerial.write((unsigned char)0);
    //     uint32_t file_size = sd_file_obj.size();
    //     BTSerial.write(file_size & 0xff);
    //     BTSerial.write((file_size >> 8) & 0xff);
    //     BTSerial.write((file_size >> 16) & 0xff);
    //     BTSerial.write((file_size >> 24) & 0xff);

    //     uint32_t bluetooth_total_sent_byte_count = 0;

    //     char read_buffer[SD_CARD_READ_BUFFER_SIZE];
    //     uint32_t total_read_count = 0;
    //     // reading file data from SD card
    //     while (1)
    //     {
    //       size_t read_count = sd_file_obj.readBytes(read_buffer, SD_CARD_READ_BUFFER_SIZE);
    //       if (read_count < 1)
    //       {
    //         break;
    //       }

    //       total_read_count += read_count;

    //       size_t total_sent_count  = 0;
    //       size_t sent_count        = 0;
    //       size_t remain_byte_count = read_count;

    //       // sending data over serial interface
    //       while (1)
    //       {
    //         Serial.print(F("BLUETOOTH_SENDING_DATA: "));
    //         Serial.print(F("FILE_SIZE: "));
    //         Serial.print(file_size);
    //         Serial.print(F(" SENT_COUNT: "));
    //         Serial.print(bluetooth_total_sent_byte_count);
    //         Serial.print(F("\n"));

    //         sent_count = BTSerial.write(read_buffer + total_sent_count, remain_byte_count);
    //         if (sent_count < 1)
    //         {
    //           break;
    //         }

    //         total_sent_count += sent_count;
    //         remain_byte_count -= sent_count;
    //         bluetooth_total_sent_byte_count += sent_count;

    //         if (remain_byte_count == 0)
    //         {
    //           break;
    //         }
    //       }

    //       if (sent_count < 1)
    //       {
    //         break;
    //       }
    //     }

    //     Serial.print(F("\n\nEND_BLUETOOTH_SENDING_DATA\n"));
    //   }
    //   else
    //   {
    //     Serial.println(F("failed to open file on SD card"));
    //     BTSerial.write((unsigned char)1);
    //   }
    Serial.println("END BT_CMD_CODE_DOWNLOAD_DATA");
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(HC05_TX, INPUT);
  pinMode(HC05_RX, OUTPUT);
  BTSerial.begin(9600);
  while(!Serial){;}
  Serial.println("setup done");
}

void loop()
{
  if (BTSerial.available())
  {
    handle_bluetooth_communication();
  }
}
