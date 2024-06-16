// Simple I2C sensor readout example.

#include <jee.h>
#include <jee/hal.h>
#include <jee/i2c-ssd1306.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard("oled");

    I2cGpio i2c;
    i2c.init(I2C_PINS, 1);

    SSD1306<decltype(i2c), true, 0x3D> oled (i2c);
    oled.init();

    // display a trivial pattern, just to verify that it works
    static uint8_t data [] = {
        255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0,255
    };
    auto t = cycles::micros();
    oled.copyBand (  0,  0, data, sizeof data);
    oled.copyBand (  1,  8, data, sizeof data);
    oled.copyBand (  2, 16, data, sizeof data);
    oled.copyBand (  3, 24, data, sizeof data);
    oled.copyBand (  4, 32, data, sizeof data);
    oled.copyBand (  5, 40, data, sizeof data);
    oled.copyBand (  6, 48, data, sizeof data);
    oled.copyBand (  7, 56, data, sizeof data);
    oled.copyBand (100,  0, data, sizeof data);
    oled.copyBand (101,  8, data, sizeof data);
    oled.copyBand (102, 16, data, sizeof data);
    oled.copyBand (103, 24, data, sizeof data);
    oled.copyBand (104, 32, data, sizeof data);
    oled.copyBand (105, 40, data, sizeof data);
    oled.copyBand (106, 48, data, sizeof data);
    oled.copyBand (107, 56, data, sizeof data);
    t = cycles::micros() - t;
    logf("oled: %d µs", t);

    while (true) {
        sys::wait(250);
        led.toggle();
    }
}
