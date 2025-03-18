#include <LedControl.h>

#define MAX7219_MAX_DEVICES 4 // 8x32 = 4 modules (each 8x8)
#define MAX7219_CLK_PIN 8
#define MAX7219_CS_PIN 7
#define MAX7219_DATA_PIN 6

LedControl lc = LedControl(MAX7219_DATA_PIN, MAX7219_CLK_PIN, MAX7219_CS_PIN, MAX7219_MAX_DEVICES); // 1 = Number of MAX7219 chips

void setup()
{
    Serial.begin(9600);

    // Initialize LED matrix
    lc.shutdown(0, false); // Wake up display
    lc.setIntensity(0, 5); // Set brightness (0-15)
    lc.clearDisplay(0);
    Serial.println("OK");
}

void loop()
{
    int base_intensity = 0;
    int device_count = lc.getDeviceCount();
    for (int device_idx = 0; device_idx < device_count; device_idx++)
    {
        lc.shutdown(device_idx, false);
        lc.setIntensity(device_idx, base_intensity + device_idx);
    }

    for (int device_idx = 0; device_idx < device_count; device_idx++)
    {
        // lc.setLed(device_idx, 0, 0, true);

        for (int i = 0; i < 8; i++)
        {
            lc.setLed(device_idx, i, 0, true);
        }

        delay(1000);
        // for(int i = 0; i < 8; i++){
        //   for(int j = 0; j < 8; j++){
        //     lc.setLed(device_idx, i, j, true);
        //     delay(100);
        //   }
        // }
    }
}
