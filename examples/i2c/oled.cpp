// Simple I2C sensor readout example.

#include <jee.h>
#include <jee/hal.h>
#include <jee/i2c-ssd1306.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard("oled");


#if 0
    I2cGpio i2c;
    i2c.init(I2C_PINS, 1);
    i2c.detect();
#else
    I2cHw<I2C_NAME.ADDR> i2c (ena::I2C_NAME, I2C_FREQ);
    i2c.init(I2C_PINS, 400);
#endif

    SSD1306<decltype(i2c), true, 0x3D> oled (i2c);
    oled.init();

    auto t = cycles::micros();
    oled.clear();
    t = cycles::micros() - t;
    logf("clear %d µs", t);

#if 1
    // display a trivial pattern, just to verify that it works
    static uint8_t data [] = {
        255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0,255
    };
    t = cycles::micros();
    oled.copyBand (  0,  0, data, sizeof data);
    oled.copyBand (  1,  8, data, sizeof data);
    oled.copyBand ( 12, 16, data, sizeof data);
    oled.copyBand ( 13, 24, data, sizeof data);
    oled.copyBand ( 24, 32, data, sizeof data);
    oled.copyBand ( 25, 40, data, sizeof data);
    oled.copyBand ( 36, 48, data, sizeof data);
    oled.copyBand ( 37, 56, data, sizeof data);
    oled.copyBand ( 70,  0, data, sizeof data);
    oled.copyBand ( 71,  8, data, sizeof data);
    oled.copyBand ( 82, 16, data, sizeof data);
    oled.copyBand ( 83, 24, data, sizeof data);
    oled.copyBand ( 94, 32, data, sizeof data);
    oled.copyBand ( 95, 40, data, sizeof data);
    oled.copyBand (106, 48, data, sizeof data);
    oled.copyBand (107, 56, data, sizeof data);
    t = cycles::micros() - t;
    logf("oled: %d µs", t);
#endif

    while (true) {
        sys::wait(250);
        led.toggle();
    }
}
