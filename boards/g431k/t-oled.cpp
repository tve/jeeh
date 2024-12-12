// I2C example with two different OLED displays.

#include <jee/dev/ssd1306.h>

enum { OLED1 = 0x3D, OLED2 = 0x3C };

void testOled () {
    // same type, two instances: one is for 128x64, the other for 128x32
    SSD1306 oled1 (i2cBus, 64);
    SSD1306 oled2 (i2cBus, 32);

    // display a trivial pattern, just to verify that it works
    static uint8_t const data [] = {
        255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0,255
    };

    i2cBus.select(OLED1);
    oled1.init();
    auto t = cycles::micros();
    oled1.clear();
    t = cycles::micros() - t;
    logf("clear 1: %5d µs", t);

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

    i2cBus.select(OLED2);
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
}
