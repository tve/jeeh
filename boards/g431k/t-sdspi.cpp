// Test the µSD card, connected via SPI.

auto& sdSpi = spiBus;

struct SdConnect : spi::Gpio {

    int init () {
        spi::Gpio::init(10);
        select(sdSel);

        mosi = 1;
        for (auto i = 0; i < 10; ++i)
            rwByte(0xFF);

        if (cmd1(0, 0, 0x95) != 1 && cmd1(0, 0, 0x95) != 1)
            return -2;

        cmd4(8, 0x000001AA, 0x87);
        cmd4(58, 0, 0xFD);

        auto start = cycles::millis();
        while (cycles::millis()-start < 500) {
            cmd1(55, 0, 0x65);
            if (cmd1(41, 1<<30, 0x77) == 0)
                break;
        }

        cmd1(16, 512); // set block length

        auto r = cmd4(58, 0, 0xFD);
        deinit();

        return r == -1 ? -1 : (r >> 30) & 1;;
    }

    void send16b (uint16_t v) {
        rwByte(v >> 8);
        rwByte(v);
    }

    int cmd (int req, uint32_t arg, uint8_t crc =0) {
        enable();
        rwByte(0xFF);
        assert(miso != 0); // verify not busy
        rwByte(0x40 | req);
        send16b(arg >> 16);
        send16b(arg);
        rwByte(crc);

        for (int i = 0; i < 10; ++i) {
            auto r = rwByte(0xFF);
            if ((r & 0x80) == 0)
                return r;
        }
        return -1;
    }

    int cmd1 (int req, uint32_t arg, uint8_t crc =0) {
        auto r = cmd(req, arg, crc);
        disable();
        rwByte(0xFF);
        //logf("cmd %2d: %d", req, r);
        return r;
    }

    int cmd4 (int req, uint32_t arg, uint8_t crc =0) {
        auto r = cmd(req, arg, crc);
        uint32_t v = 0;
        for (auto i = 0; i < 4; ++i)
            v = (v<<8) | rwByte(0xFF);
        disable();
        rwByte(0xFF);
        logf("cmd %2d: %d -> %08x", req, r, v);
        return r <= 1 ? v : -1;
    }
};

bool quiet = false;

int cmd (uint8_t req, uint32_t arg) {
    uint8_t dat [7];
    dat[0] = 0xFF;
    dat[1] = 0x40 | req;
    dat[2] = arg >> 24;
    dat[3] = arg >> 16;
    dat[4] = arg >> 8;
    dat[5] = arg;
    dat[6] = 0;
    sdSpi.enable();
    auto start = cycles::micros();
    while (sdSpi.transfer(true, dat, 1) != 0xFF)
        if (cycles::micros()-start > 250'000) {
            sdSpi.disable();
            return -2;
        }
    if (!quiet)
        logf("busy %d us", cycles::micros()-start);

    sdSpi.transfer(true, dat, sizeof dat);
    for (int i = 0; i < 10; ++i) {
        auto r = sdSpi.transfer(true, dat, 1);
        if (0 && !quiet)
            logf("r %02x", r);
        if ((r & 0x80) == 0)
            return r;
    }
    return -1;
}

bool read512 (uint32_t blk, void* buf) {
    if (!quiet)
        logf("read 0x%x", blk);
    auto x = cmd(17, blk);
    if (0 && !quiet)
        logf("x %02x", x);

    uint8_t dat [] = { 0xFF, 0xFF, 0xFF };

    auto start = cycles::micros();
    while (cycles::micros()-start < 100'000) {
        auto r = sdSpi.transfer(true, dat, 1); // token
        if (r == 0xFE)
            break;
    }
    if (!quiet)
        logf("rwait %d us", cycles::micros()-start);

    start = cycles::micros();
    sdSpi.transfer(false, (uint8_t*) buf, 512);
    if (!quiet)
        logf("rxfer %d us", cycles::micros()-start);

    sdSpi.transfer(true, dat, 3); // crc
    if (!quiet)
        logf("c %02x %02x", dat[0], dat[1]);

    sdSpi.disable();
    sdSpi.transfer(true, dat, 1);
    return true;
}

bool write512 (uint32_t blk, void* buf) {
    if (!quiet)
        logf("write 0x%x", blk);
    auto y = cmd(24, blk);
    if (0 && !quiet)
        logf("y %02x", y);

    uint8_t dat [] = { 0xFF, 0xFE, 0xFF };

    auto start = cycles::micros();
    sdSpi.transfer(true, dat, 2);
    sdSpi.transfer(true, (uint8_t*) buf, 512);
    auto r = sdSpi.transfer(true, dat, 3);
    if (!quiet)
        logf("wxfer %d us", cycles::micros()-start);

#if 0
    start = cycles::millis();
    while (sdSpi.transfer(true, dat, 1) != 0xFF)
        if (cycles::millis()-start > 1000) {
            logf("busy?");
            r = 0;
            break;
        }
#endif

    sdSpi.disable();
    sdSpi.transfer(true, dat, 1);
    if ((r & 0x1F) != 0x05)
        logf("W? %02x", r);
    return (r & 0x1F) == 0x05;
}

void testSdSpi () {
    sdSpi.deinit(); // need to reinit after GPIO slow mode completes

    // see http://elm-chan.org/docs/mmc/mmc_e.html
    // and https://electronics.stackexchange.com/questions/602105

    SdConnect sdc;
    auto start = cycles::micros();
    auto r = sdc.init();
    auto bsh = r ? 0 : 9;
    logf("sdhc %d, bsh %d, %d us", r, bsh, cycles::micros()-start);

#if STM32F3
    sdSpi.init(10'000);
#else
    sdSpi.init(40'000);
#endif
    sdSpi.select(sdSel);

    uint8_t buf [512];
    for (auto i = 0; i < 3; ++i) {
        memset(buf, 0, sizeof buf);
        read512(i << bsh, buf);
        logDump(buf, sizeof buf);
    }

    auto blk = 1 << bsh;
    read512(blk, buf);
    logDump(buf, 16);
    for (auto i = 0; i < 10; ++i)
        buf[2*i] += i;
    write512(blk, buf);
    memset(buf, 0, sizeof buf);
    read512(blk, buf);
    logDump(buf, 16);

    quiet = true;

    start = cycles::millis();
    for (auto i = 0; i < 2048; ++i)
        if (!read512(i << bsh, buf))
            logf("r? #%d", i);
    logf("read 1 MB in %d ms", cycles::millis()-start);

    start = cycles::millis();
    for (auto i = 0; i < 2048; ++i)
        if (!write512((1000+i) << bsh, buf))
            logf("w? #%d", i);
    logf("write 1 MB in %d ms", cycles::millis()-start);

    //quiet = false;
    memset(buf, 0, sizeof buf);
    read512(blk, buf);
    logDump(buf, 16);
}
