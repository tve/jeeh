// Scan the I2C bus to detect all attached devices.

void showPresence (char const* name, uint8_t addr) {
    i2cBus.select(addr);
    bool ack = i2cBus.write(nullptr, 0) >= 0;
    logf("%15s @ 0x%02x: %s", name, addr, ack ? "OK" : "NOT FOUND");
}

void testScan () {
    bmpVcc = 1;
    i2c::detect(i2cBus);

    showPresence("HMC5883", 0x1E);
    showPresence("128x32 OLED", 0x3C);
    showPresence("128x64 OLED", 0x3D);
    showPresence("SHT21", 0x40);
    showPresence("32 KB FRAM", 0x50);
    showPresence("ADXL345", 0x53);
    showPresence("ITG3200", 0x68);
    showPresence("BMP390", 0x77);
}
