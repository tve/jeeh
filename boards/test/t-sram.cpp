// Test SRAM connected via SPI.

auto& sram = spiBus;

void rdMem (uint16_t addr, void* buf, uint16_t len) {
    uint8_t hdr [] = { 0x03, (uint8_t) (addr>>8), (uint8_t) addr };
    sram.ioReq("<W"_IO, hdr, sizeof hdr);
    sram.ioReq("R>"_IO, (uint8_t*) buf, len);
}

void wrMem (uint16_t addr, void const* buf, uint16_t len) {
    uint8_t hdr [] = { 0x02, (uint8_t) (addr>>8), (uint8_t) addr };
    sram.ioReq("<W"_IO, hdr, sizeof hdr);
    sram.ioReq("W>"_IO, (uint8_t*) buf, len);
}

void testSram () {
    spiSelect(sram, sramSel);
    cycles::msBusy(10);
uint8_t z [10] = {};
sram.ioReq("W"_IO, z, sizeof z);

    uint8_t hdr [] = { 0x01, 0x40 }; // write status, sequential mode
    sram.ioReq("<W>"_IO, hdr, sizeof hdr);

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
