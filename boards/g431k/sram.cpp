#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

//spi::Gpio sram;
spi::Poll<SPI1.ADDR> sram (ena::SPI1, 100);

void rdMem (uint16_t addr, void* buf, uint16_t len) {
    sram.enable();
    sram.rwByte(0x03); // read
    sram.rwByte(addr >> 8);
    sram.rwByte(addr);
    sram.transfer(false, (uint8_t*) buf, len);
    sram.disable();
}

void wrMem (uint16_t addr, void const* buf, uint16_t len) {
    sram.enable();
    sram.rwByte(0x02); // write
    sram.rwByte(addr >> 8);
    sram.rwByte(addr);
    sram.transfer(true, (uint8_t*) buf, len);
    sram.disable();
}

int main () {
    initBoard();
    sram.init(SPI_PINS, 10'000);
    spiSelect(sram, sramSel);

    sram.enable();
    sram.rwByte(0x01); // write status
    sram.rwByte(0x40); // sequential mode
    sram.disable();

    uint8_t buf [2][32];
    memset(buf, 0, sizeof buf);
    wrMem(0, buf, sizeof buf);

    wrMem(30, "hello", 5);
    wrMem(40, "world", 5);
    rdMem(20, buf[0], sizeof buf[0]);
    wrMem(35, "-spi-", 5);
    rdMem(20, buf[1], sizeof buf[1]);

    logDump(buf, sizeof buf);

    while (true) { led.toggle(); cycles::msBusy(500); }
}
