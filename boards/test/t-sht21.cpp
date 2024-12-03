// Test the attached SHT21 sensor over I2C.

enum { SHT21 = 0x40 }; // I2C address

// start a measurement, wait until ready, get 3 result bytes
uint16_t measure (uint8_t type, uint8_t delay) {
#if MODE_GPIO
    i2cBus.start(SHT21*2);
    i2cBus.wrByte(type);
    i2cBus.stop();

    cycles::msBusy(delay);

    uint8_t buf [3];
    i2cBus.start(SHT21*2+1);
    buf[0] = i2cBus.rdByte(false);
    buf[1] = i2cBus.rdByte(false);
    buf[2] = i2cBus.rdByte(true); // last one
    //logDump(buf, sizeof buf);

    return (buf[0] << 8) | (buf[1] & ~3); // clear lower 2 bits
#else
    (void) type; (void) delay;
    return 0;
#endif
}

void testSht21 () {
    for (auto i = 0; i < 3; ++i) {
        auto tRaw = measure(0xF3, 85); // measure temperature
        auto hRaw = measure(0xF5, 29); // measure humidity
        auto t = (17572 * tRaw >> 16) - 4685; // °C x 100
        auto h = (12500 * hRaw >> 16) - 600; // %RH x 100
        logf("T: %d.%02d C, RH: %d.%02d %%", t/100, t%100, h/100, h%100);
    }
}
