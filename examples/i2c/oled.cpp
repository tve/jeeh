// I2C example with two different OLED displays.

#include <jee.h>
#include <jee/hal.h>
#include <jee/i2c-ssd1306.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard("oled");

    // one I2C bus, two I2C devices
#if 0
    I2cHw<I2C_NAME.ADDR> i2c (ena::I2C_NAME, I2C_FREQ);
#else
    I2cDma<I2C_TYPE> i2c (I2C_CONF);
#endif
    BusDev dev1 (i2c, 0x3D);
    BusDev dev2 (i2c, 0x3C);

    // same type, two instances: one is for 128x64, the other for 128x32
    SSD1306 oled1 (dev1, 64);
    SSD1306 oled2 (dev2, 32);

    i2c.init(I2C_PINS, 1000);

    oled1.init();
    auto t = cycles::micros();
    oled1.clear();
    t = cycles::micros() - t;
    logf("clear 1: %5d µs", t);

    // display a trivial pattern, just to verify that it works
    static uint8_t const data [] = {
        255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0,255
    };

    t = cycles::micros();
    oled1.copyBand (  0,  0, data, sizeof data);
    oled1.copyBand (  1,  8, data, sizeof data);
    oled1.copyBand ( 12, 16, data, sizeof data);
    oled1.copyBand ( 13, 24, data, sizeof data);
    oled1.copyBand ( 24, 32, data, sizeof data);
    oled1.copyBand ( 25, 40, data, sizeof data);
    oled1.copyBand ( 36, 48, data, sizeof data);
    oled1.copyBand ( 37, 56, data, sizeof data);
    oled1.copyBand ( 70,  0, data, sizeof data);
    oled1.copyBand ( 71,  8, data, sizeof data);
    oled1.copyBand ( 82, 16, data, sizeof data);
    oled1.copyBand ( 83, 24, data, sizeof data);
    oled1.copyBand ( 94, 32, data, sizeof data);
    oled1.copyBand ( 95, 40, data, sizeof data);
    oled1.copyBand (106, 48, data, sizeof data);
    oled1.copyBand (107, 56, data, sizeof data);
    t = cycles::micros() - t;
    logf(" oled 1: %5d µs", t);

    oled2.init();
    auto t2 = cycles::micros();
    oled2.clear();
    t2 = cycles::micros() - t2;
    logf("clear 2: %5d µs", t2);

    t2 = cycles::micros();
    oled2.copyBand (  0,  0, data, sizeof data);
    oled2.copyBand (  1,  8, data, sizeof data);
    oled2.copyBand ( 12, 16, data, sizeof data);
    oled2.copyBand ( 13, 24, data, sizeof data);
    oled2.copyBand ( 94,  0, data, sizeof data);
    oled2.copyBand ( 95,  8, data, sizeof data);
    oled2.copyBand (106, 16, data, sizeof data);
    oled2.copyBand (107, 24, data, sizeof data);
    t2 = cycles::micros() - t2;
    logf(" oled 2: %5d µs", t2);

    while (true) {
        sys::wait(250);
        led.toggle();
    }
}
