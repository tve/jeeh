// Test the attached FRAM over I2C.

constexpr auto N = 5;

void readN (uint16_t addr, uint8_t* ptr) {
    auto ok = i2cBus.readRegs16(addr, ptr, N);
    assert(ok);
}

void writeN (uint16_t addr, uint8_t const* ptr) {
    auto ok = i2cBus.writeRegs16(addr, ptr, N);
    assert(ok);
}

void testFram () {
    i2cBus.select(0x50);

    uint8_t buf [N];
    memset(buf, 0xEE, sizeof buf);
    for (auto i = 0; i < 3; ++i)
        writeN(N*i, buf);
    cycles::usBusy(40); // TODO ???

    for (auto i = 0; i < 3; ++i) {
        memset(buf, 0x55, sizeof buf);
        readN(N*i, buf);
        //logDump(buf, 16);
        logf("%02x %02x ... %02x %02x", buf[0], buf[1], buf[N-2], buf[N-1]);
    }
    cycles::usBusy(40); // TODO ???

    for (auto i = 0; i < 2; ++i) {
        memset(buf, i+128, sizeof buf);
        writeN(N*i, buf);
    }
    cycles::usBusy(40); // TODO ???

    for (auto i = 0; i < 2; ++i) {
        memset(buf, 0xAA, sizeof buf);
        readN(N*i+2, buf);
        //logDump(buf, 16);
        logf("%02x %02x ... %02x %02x", buf[0], buf[1], buf[N-2], buf[N-1]);
    }

    buf[0] = i2cBus.readReg16(0);
    buf[1] = i2cBus.readReg16(N);
    buf[2] = i2cBus.readReg16(2*N+3);
    logf("%02x %02x %02x", buf[0], buf[1], buf[2]);
}
