// Test the attached 9-DOF IMU board via I2C.

enum { HMC5883 = 0x1E, ADXL345 = 0x53, ITG3200 = 0x68 };

void testImu () {
    uint8_t buf [6];

    i2cBus.select(HMC5883);
    i2cBus.writeReg(0, 0x18); // 75 Hz
    i2cBus.writeReg(1, 0x40); // +/- 1.9 Ga
    i2cBus.writeReg(2, 0x00); // continuous

    i2cBus.select(ADXL345);
    i2cBus.writeReg(0x2D, 0x08); // Measurement mode
    i2cBus.writeReg(0x31, 0x08); // full resolution
    i2cBus.writeReg(0x2C, 0x09); // 50 Hz rate

    i2cBus.select(ITG3200);
    i2cBus.writeReg(0x3E, 0x80); // full-scale range, 42 Hz
    i2cBus.writeReg(0x16, 0x1B); // DLPF_CFG=3, FS_SEL=3
    i2cBus.writeReg(0x15, 0x0A); // SMPLRT_DIV = 10 (50Hz)
    i2cBus.writeReg(0x3E, 0x03); // PLL with z gyro ref

    i2cBus.select(HMC5883);
    for (auto i = 0; i < 4; ++i) {
        cycles::msBusy(25);
        i2cBus.readRegs(0x03, buf, sizeof buf);

        int16_t x = (buf[0] << 8) | buf[1];
        int16_t y = (buf[2] << 8) | buf[3];
        int16_t z = (buf[4] << 8) | buf[5];
        if (i > 0)
            logf("hmc5883 compass: xyz = %6d %6d %6d", x, y, z);
    }

    i2cBus.select(ADXL345);
    for (auto i = 0; i < 4; ++i) {
        cycles::msBusy(25);
        i2cBus.readRegs(0x32, buf, sizeof buf);

        int16_t x = (buf[1] << 8) | buf[0];
        int16_t y = (buf[3] << 8) | buf[2];
        int16_t z = (buf[5] << 8) | buf[4];
        if (i > 0)
            logf("adxl345 accel:   xyz = %6d %6d %6d", x, y, z);
    }

    i2cBus.select(ITG3200);
    for (auto i = 0; i < 4; ++i) {
        cycles::msBusy(25);
        i2cBus.readRegs(0x1D, buf, sizeof buf);

        int16_t x = (buf[0] << 8) | buf[1];
        int16_t y = (buf[2] << 8) | buf[3];
        int16_t z = (buf[4] << 8) | buf[5];
        if (i > 0)
            logf("itg3200 gyro:    xyz = %6d %6d %6d", x, y, z);
    }
}
