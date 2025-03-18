// LedControl@1.0.6
// https://github.com/wayoda/LedControl
#include <LedControl.h>

#define MAX7219_MAX_DEVICES 4 // 8x32 = 4 modules (each 8x8)
#define MAX7219_CLK_PIN 8
#define MAX7219_CS_PIN 7
#define MAX7219_DATA_PIN 6

LedControl lc = LedControl(MAX7219_DATA_PIN, MAX7219_CLK_PIN, MAX7219_CS_PIN, MAX7219_MAX_DEVICES); // 1 = Number of MAX7219 chips

void setup()
{
    Serial.begin(9600);

    while (!Serial)
    {
        // wait for serial port to connect. Needed for native USB port only
    }
    // Initialize LED matrix
    lc.shutdown(0, false); // Wake up display
    lc.setIntensity(0, 5); // Set brightness (0-15)
    lc.clearDisplay(0);
}

void loop()
{
    displayNumber(2222);
    delay(1000);
}

// Function to display a 4-digit number
void displayNumber(int num)
{
    for (int i = 0; i < 4; i++)
    {
        lc.setDigit(0, i, num % 10, false);
        num /= 10;
    }
}

// Sketch uses 2772 bytes (8%) of program storage space. Maximum is 32256 bytes.
// Global variables use 272 bytes (13%) of dynamic memory, leaving 1776 bytes for local variables. Maximum is 2048 bytes.
