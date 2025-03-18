// MD_Parola@3.7.3
// MD_MAX72XX@3.5.1
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
// define LED Matrix setup
#define MAX7219_HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX7219_MAX_DEVICES 4 // 8x32 = 4 modules (each 8x8)

#define MAX7219_CLK_PIN 8
#define MAX7219_CS_PIN 7
#define MAX7219_DATA_PIN 6

// MAX7219:CLK -> R3:8
// MAX7219:CS -> R3:7
// MAX7219:DIN -> R3:6
// initialize LED matrix
MD_Parola matrix = MD_Parola(
    MAX7219_HARDWARE_TYPE,
    MAX7219_DATA_PIN,
    MAX7219_CLK_PIN,
    MAX7219_CS_PIN,
    MAX7219_MAX_DEVICES);

void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
        // wait for serial port to connect. Needed for native USB port only
    }

    // initialize LED matrix
    matrix.begin();
    // matrix.setIntensity(2);  // Adjust brightness (0-15)
    matrix.setIntensity(1); // Adjust brightness (0-15)
    matrix.displayClear();
}

void loop()
{
    matrix.displayClear();
    matrix.displayText("aika", PA_CENTER, 50, 1000, PA_SCROLL_LEFT);
    while (!matrix.displayAnimate())
        ; // Wait for scrolling to finish

    delay(100);
}

// Sketch uses 20902 bytes (64%) of program storage space. Maximum is 32256 bytes.
// Global variables use 274 bytes (13%) of dynamic memory, leaving 1774 bytes for local variables. Maximum is 2048 bytes.
