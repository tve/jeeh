// Test the attached SHT21 sensor over I2C.

enum { SHT21 = 0x40 }; // I2C address

// start a measurement, wait until ready, get 3 result bytes
uint16_t measure (uint8_t type, uint8_t delay) {
    i2cBus.write(&type, 1);

    cycles::msBusy(delay);

    uint8_t buf [3];
    i2cBus.read(buf, sizeof buf);
    //logDump(buf, sizeof buf);

    return (buf[0] << 8) | (buf[1] & ~3); // clear lower 2 bits
}

void testSht21 () {
    i2cBus.select(SHT21);

    for (auto i = 0; i < 3; ++i) {
        auto tRaw = measure(0xF3, 85); // measure temperature
        auto hRaw = measure(0xF5, 29); // measure humidity
        auto t = (17572 * tRaw >> 16) - 4685; // °C x 100
        auto h = (12500 * hRaw >> 16) - 600; // %RH x 100
        logf("T: %d.%02d C, RH: %d.%02d %%", t/100, t%100, h/100, h%100);
    }
}
