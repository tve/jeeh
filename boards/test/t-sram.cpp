// Test SRAM connected via SPI.

auto& sram = spiBus;

#if 0
void rdMem (uint16_t addr, void* buf, uint16_t len) {
    uint8_t hdr [] = { 0x03, (uint8_t) (addr>>8), (uint8_t) addr };
    sram.ioRequest(IO_START|IO_WRITE, hdr, sizeof hdr);
    sram.ioRequest(IO_READ|IO_STOP, (uint8_t*) buf, len);
}

void wrMem (uint16_t addr, void const* buf, uint16_t len) {
    uint8_t hdr [] = { 0x02, (uint8_t) (addr>>8), (uint8_t) addr };
    sram.ioRequest(IO_START|IO_WRITE, hdr, sizeof hdr);
    sram.ioRequest(IO_WRITE|IO_STOP, (uint8_t*) buf, len);
}
#else
void rdMem (uint16_t addr, void* buf, uint16_t len) {
    uint8_t hdr [] = { 0x03, (uint8_t) (addr>>8), (uint8_t) addr };
    IoReq const req [] = {
        { IO_START|IO_WRITE, sizeof hdr, hdr },
        { IO_READ|IO_STOP, len, (uint8_t*) buf },
    };
    sram.ioRequest(req);
}

void wrMem (uint16_t addr, void const* buf, uint16_t len) {
    uint8_t hdr [] = { 0x02, (uint8_t) (addr>>8), (uint8_t) addr };
     IoReq const req [] = {
        { IO_START|IO_WRITE, sizeof hdr, hdr },
        { IO_WRITE|IO_STOP, len, (uint8_t*) buf },
    };
    sram.ioRequest(req);
}
#endif

void testSram () {
    spiSelect(sram, sramSel);
    cycles::msBusy(10);

    uint8_t hdr [] = { 0x01, 0x40 }; // write status, sequential mode
    sram.ioRequest(IO_START|IO_WRITE|IO_STOP, hdr, sizeof hdr);

    uint8_t buf [2][32];
    memset(buf, 0, sizeof buf);
    wrMem(0, buf, sizeof buf);

    wrMem(30, "hello", 5);
    wrMem(40, "world", 5);
    rdMem(20, buf[0], sizeof buf[0]);
    wrMem(35, "-spi-", 5);
    rdMem(20, buf[1], sizeof buf[1]);

    logDump(buf, sizeof buf);
}
