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
    hardFaulter = hardFaultHandler;
    fastClock();
    cycles::init();

    // the UART config comes from platformio.ini and is defined in defs.h
    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });

    printf("???\r%s @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);
    led.mode("P"); // push-pull output

    I2cGpio i2c;
    printf("SCL=PB6, SDA=PB7\n");
    i2c.init("B6,B7");
    i2c.detect(); // look for audio codec

    uint8_t buf [6];

    i2c.writeReg(HMC5883, 0, 0x18); sys::wait(10); // 75 Hz
    i2c.writeReg(HMC5883, 1, 0x40); sys::wait(10); // +/- 1.9 Ga
    i2c.writeReg(HMC5883, 2, 0x00); sys::wait(10); // continuous

    for (auto i = 0; i < 3; ++i) {
        sys::wait(50);
        i2c.readRegs(HMC5883, 3, buf, sizeof buf);

        auto x = (int16_t) ((buf[0] << 8) | buf[1]);
        auto y = (int16_t) ((buf[2] << 8) | buf[3]);
        auto z = (int16_t) ((buf[4] << 8) | buf[5]);
        printf("HMC5883 compass: xyz = %d %d %d\n", x, y, z);
    }

    i2c.writeReg(ADXL345, 0x2D, 0x08); sys::wait(10); // Measurement mode
    i2c.writeReg(ADXL345, 0x31, 0x08); sys::wait(10); // full resolution
    i2c.writeReg(ADXL345, 0x2C, 0x09); sys::wait(10); // 50 Hz rate

    for (auto i = 0; i < 3; ++i) {
        sys::wait(50);
        i2c.readRegs(ADXL345, 0x32, buf, sizeof buf);

        auto x = ((int8_t) buf[1] << 8) | buf[0];
        auto y = ((int8_t) buf[3] << 8) | buf[2];
        auto z = ((int8_t) buf[5] << 8) | buf[4];
        printf("ADXL345 accel:   xyz = %d %d %d\n", x, y, z);
    }

    i2c.writeReg(ITG3200, 0x3E, 0x80); sys::wait(10); // full-scale range, 42 Hz
    i2c.writeReg(ITG3200, 0x16, 0x1B); sys::wait(10); // DLPF_CFG=3, FS_SEL=3
    i2c.writeReg(ITG3200, 0x15, 0x0A); sys::wait(10); // SMPLRT_DIV = 10 (50Hz)
    i2c.writeReg(ITG3200, 0x3E, 0x03); sys::wait(10); // PLL with z gyro ref

    for (auto i = 0; i < 3; ++i) {
        sys::wait(50);
        i2c.readRegs(ITG3200, 0x1D, buf, sizeof buf);

        auto x = (int16_t) ((buf[0] << 8) | buf[1]);
        auto y = (int16_t) ((buf[2] << 8) | buf[3]);
        auto z = (int16_t) ((buf[4] << 8) | buf[5]);
        printf("ITG3200 gyro:    xyz = %d %d %d\n", x, y, z);
    }

    SSD1306 oled (i2c);
    oled.init();

    // display a trivial pattern, just to verify that it works
    static uint8_t data [] = {
        255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0,255,0,255
    };
    oled.copyBand (  0,  0, data, sizeof data);
    oled.copyBand (  1,  8, data, sizeof data);
    oled.copyBand (  2, 16, data, sizeof data);
    oled.copyBand (  3, 24, data, sizeof data);
    oled.copyBand (100,  0, data, sizeof data);
    oled.copyBand (101,  8, data, sizeof data);
    oled.copyBand (102, 16, data, sizeof data);
    oled.copyBand (103, 24, data, sizeof data);

    while (true) {
        sys::wait(250);
        led.toggle();
    }
}
