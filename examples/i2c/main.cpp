// Simple I2C sensor readout example.

#include <jee.h>
#include <jee/hal.h>
#include <jee/i2c-ssd1306.h>
using namespace jeeh;
#include "defs.h"

enum { HMC5883=0x1E, ADXL345=0x53, ITG3200=0x68 };

constexpr Pin led ("B3"); // Nucleo-L432KC

Uart uart ('U'); // DMA/IRQ-based UART device driver, id = 'U'

extern "C" int _write (int, char* ptr, int len) {
    Message m { 'U', 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

int main () {
    fastClock();
    cycles::init();

    // the UART config comes from platformio.ini and is defined in defs.h
    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });

    printf("\n%s @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);
    led.mode("P"); // push-pull output

    I2cGpio i2c;
    printf("SCL=PB6, SDA=PB7\n");
    i2c.init("B6,B7", 1);
    i2c.detect(); // look for audio codec

    uint8_t buf [6];

    i2c.writeReg(HMC5883, 0, 0x18); // 75 Hz
    i2c.writeReg(HMC5883, 1, 0x40); // +/- 1.9 Ga
    i2c.writeReg(HMC5883, 2, 0x00); // continuous

    i2c.writeReg(ADXL345, 0x2D, 0x08); // Measurement mode
    i2c.writeReg(ADXL345, 0x31, 0x08); // full resolution
    i2c.writeReg(ADXL345, 0x2C, 0x09); // 50 Hz rate

    i2c.writeReg(ITG3200, 0x3E, 0x80); // full-scale range, 42 Hz
    i2c.writeReg(ITG3200, 0x16, 0x1B); // DLPF_CFG=3, FS_SEL=3
    i2c.writeReg(ITG3200, 0x15, 0x0A); // SMPLRT_DIV = 10 (50Hz)
    i2c.writeReg(ITG3200, 0x3E, 0x03); // PLL with z gyro ref

    for (auto i = 0; i < 4; ++i) {
        sys::wait(25);
        i2c.readRegs(HMC5883, 3, buf, sizeof buf);

        auto x = (int16_t) ((buf[0] << 8) | buf[1]);
        auto y = (int16_t) ((buf[2] << 8) | buf[3]);
        auto z = (int16_t) ((buf[4] << 8) | buf[5]);
        if (i > 0)
            printf("HMC5883 compass: xyz = %4d %4d %4d\n", x, y, z);
    }

    for (auto i = 0; i < 4; ++i) {
        sys::wait(25);
        i2c.readRegs(ADXL345, 0x32, buf, sizeof buf);

        auto x = ((int8_t) buf[1] << 8) | buf[0];
        auto y = ((int8_t) buf[3] << 8) | buf[2];
        auto z = ((int8_t) buf[5] << 8) | buf[4];
        if (i > 0)
            printf("ADXL345 accel:   xyz = %4d %4d %4d\n", x, y, z);
    }

    for (auto i = 0; i < 4; ++i) {
        sys::wait(25);
        i2c.readRegs(ITG3200, 0x1D, buf, sizeof buf);

        auto x = (int16_t) ((buf[0] << 8) | buf[1]);
        auto y = (int16_t) ((buf[2] << 8) | buf[3]);
        auto z = (int16_t) ((buf[4] << 8) | buf[5]);
        if (i > 0)
            printf("ITG3200 gyro:    xyz = %4d %4d %4d\n", x, y, z);
    }

    SSD1306 oled1 (i2c);
    oled1.init();

    // display a trivial pattern, just to verify that it works
    static uint8_t data1 [] = {
        255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0,255
    };
    auto t1 = cycles::count();
    oled1.copyBand (  0,  0, data1, sizeof data1);
    oled1.copyBand (  1,  8, data1, sizeof data1);
    oled1.copyBand (  2, 16, data1, sizeof data1);
    oled1.copyBand (  3, 24, data1, sizeof data1);
    oled1.copyBand (100,  0, data1, sizeof data1);
    oled1.copyBand (101,  8, data1, sizeof data1);
    oled1.copyBand (102, 16, data1, sizeof data1);
    oled1.copyBand (103, 24, data1, sizeof data1);
    t1 = cycles::count() - t1;
    printf("oled1 %d µs\n", t1 / (SystemCoreClock/1'000'000));

    SSD1306<decltype(i2c), true, 0x3D> oled2 (i2c);
    oled2.init();

    // display a trivial pattern, just to verify that it works
    static uint8_t data2 [] = {
        255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0,255
    };
    auto t2 = cycles::count();
    oled2.copyBand (  0,  0, data2, sizeof data2);
    oled2.copyBand (  1,  8, data2, sizeof data2);
    oled2.copyBand (  2, 16, data2, sizeof data2);
    oled2.copyBand (  3, 24, data2, sizeof data2);
    oled2.copyBand (  4, 32, data2, sizeof data2);
    oled2.copyBand (  5, 40, data2, sizeof data2);
    oled2.copyBand (  6, 48, data2, sizeof data2);
    oled2.copyBand (  7, 56, data2, sizeof data2);
    oled2.copyBand (100,  0, data2, sizeof data2);
    oled2.copyBand (101,  8, data2, sizeof data2);
    oled2.copyBand (102, 16, data2, sizeof data2);
    oled2.copyBand (103, 24, data2, sizeof data2);
    oled2.copyBand (104, 32, data2, sizeof data2);
    oled2.copyBand (105, 40, data2, sizeof data2);
    oled2.copyBand (106, 48, data2, sizeof data2);
    oled2.copyBand (107, 56, data2, sizeof data2);
    t2 = cycles::count() - t2;
    printf("oled2 %d µs\n", t2 / (SystemCoreClock/1'000'000));

    while (true) {
        sys::wait(250);
        led.toggle();
    }
}
