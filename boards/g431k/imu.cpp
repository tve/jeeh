// Simple I2C sensor readout example.

#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
#include <jee/i2c.h>
using namespace jeeh;
#include "defs.h"

i2c::Gpio i2cBus;
//i2c::Poll<I2C_NAME.ADDR> i2cBus (ena::I2C_NAME, I2C_FREQ);

int main () {
    initBoard();
    i2cBus.init(I2C_PINS, 400);

    // three I2C devices
    i2c::Dev hmc5883 {i2cBus, 0x1E};
    i2c::Dev adxl345 {i2cBus, 0x53};
    i2c::Dev itg3200 {i2cBus, 0x68};

    uint8_t buf [6];

    hmc5883.write(0, 0x18); // 75 Hz
    hmc5883.write(1, 0x40); // +/- 1.9 Ga
    hmc5883.write(2, 0x00); // continuous

    adxl345.write(0x2D, 0x08); // Measurement mode
    adxl345.write(0x31, 0x08); // full resolution
    adxl345.write(0x2C, 0x09); // 50 Hz rate

    itg3200.write(0x3E, 0x80); // full-scale range, 42 Hz
    itg3200.write(0x16, 0x1B); // DLPF_CFG=3, FS_SEL=3
    itg3200.write(0x15, 0x0A); // SMPLRT_DIV = 10 (50Hz)
    itg3200.write(0x3E, 0x03); // PLL with z gyro ref

    for (auto i = 0; i < 4; ++i) {
        cycles::msBusy(25);
        hmc5883.read(3, buf, sizeof buf);

        auto x = (int16_t) ((buf[0] << 8) | buf[1]);
        auto y = (int16_t) ((buf[2] << 8) | buf[3]);
        auto z = (int16_t) ((buf[4] << 8) | buf[5]);
        if (i > 0)
            logf("hmc5883 compass: xyz = %6d %6d %6d", x, y, z);
    }

    for (auto i = 0; i < 4; ++i) {
        cycles::msBusy(25);
        adxl345.read(0x32, buf, sizeof buf);

        auto x = ((int8_t) buf[1] << 8) | buf[0];
        auto y = ((int8_t) buf[3] << 8) | buf[2];
        auto z = ((int8_t) buf[5] << 8) | buf[4];
        if (i > 0)
            logf("adxl345 accel:   xyz = %6d %6d %6d", x, y, z);
    }

    for (auto i = 0; i < 4; ++i) {
        cycles::msBusy(25);
        itg3200.read(0x1D, buf, sizeof buf);

        auto x = (int16_t) ((buf[0] << 8) | buf[1]);
        auto y = (int16_t) ((buf[2] << 8) | buf[3]);
        auto z = (int16_t) ((buf[4] << 8) | buf[5]);
        if (i > 0)
            logf("itg3200 gyro:    xyz = %6d %6d %6d", x, y, z);
    }

    while (true) { cycles::msBusy(500); led.toggle(); }
}
