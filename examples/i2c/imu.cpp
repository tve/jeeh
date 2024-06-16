// Simple I2C sensor readout example.

#include <jee.h>
#include <jee/hal.h>
#include <jee/i2c-ssd1306.h>
using namespace jeeh;
#include "defs.h"

enum { HMC5883=0x1E, ADXL345=0x53, ITG3200=0x68 };

int main () {
    initBoard("imu");

    I2cGpio i2c;
    i2c.init(I2C_PINS, 1);

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
            logf("HMC5883 compass: xyz = %4d %4d %4d", x, y, z);
    }

    for (auto i = 0; i < 4; ++i) {
        sys::wait(25);
        i2c.readRegs(ADXL345, 0x32, buf, sizeof buf);

        auto x = ((int8_t) buf[1] << 8) | buf[0];
        auto y = ((int8_t) buf[3] << 8) | buf[2];
        auto z = ((int8_t) buf[5] << 8) | buf[4];
        if (i > 0)
            logf("ADXL345 accel:   xyz = %4d %4d %4d", x, y, z);
    }

    for (auto i = 0; i < 4; ++i) {
        sys::wait(25);
        i2c.readRegs(ITG3200, 0x1D, buf, sizeof buf);

        auto x = (int16_t) ((buf[0] << 8) | buf[1]);
        auto y = (int16_t) ((buf[2] << 8) | buf[3]);
        auto z = (int16_t) ((buf[4] << 8) | buf[5]);
        if (i > 0)
            logf("ITG3200 gyro:    xyz = %4d %4d %4d", x, y, z);
    }

    while (true) {
        sys::wait(250);
        led.toggle();
    }
}
