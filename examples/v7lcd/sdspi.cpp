#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
#include <jee/spi.h>
using namespace jeeh;
#include "defs.h"

spi::Gpio spiGpio;
spi::Poll<SPI_NAME.ADDR> sdSpi (ena::SPI_NAME, SPI_FREQ);

void send16b (uint16_t v) {
    spiGpio.rwByte(v >> 8);
    spiGpio.rwByte(v);
}

int cmd (int req, uint32_t arg, uint8_t crc =0) {
    spiGpio.enable();
    send16b(0xFF40 | req);
    send16b(arg >> 16);
    send16b(arg);
    spiGpio.rwByte(crc);

    for (int i = 0; i < 10; ++i) {
        auto r = spiGpio.rwByte(0xFF);
        if ((r & 0x80) == 0)
            return r;
    }
    return -1;
}

int cmd1 (int req, uint32_t arg, uint8_t crc =0) {
    auto r = cmd(req, arg, crc);
    spiGpio.disable();
    logf("cmd%3d %d", req, r);
    return r;
}

int cmd4 (int req, uint32_t arg, uint8_t crc =0) {
    auto r = cmd(req, arg, crc);
    uint8_t v = 0;
    for (auto i = 0; i < 4; ++i)
        v = (v<<8) | spiGpio.rwByte(0xFF);
    spiGpio.disable();
    logf("cmd%3d %d %08x", req, r, v);
    return r <= 1 ? v : -1;
}

int main () {
    initBoard();

    // see http://elm-chan.org/docs/mmc/mmc_e.html
    // and https://electronics.stackexchange.com/questions/602105

    auto start = cycles::millis();

    spiGpio.init(SPI_PINS, 10);
    logf("rate %d", spiGpio.rate);

    spiGpio.mosi = 1;
    for (auto i = 0; i < 10; ++i)
        spiGpio.rwByte(0xFF);

    if (cmd1(0, 0, 0x95) != 1)
        cmd1(0, 0, 0x95);

    cmd4(8, 0x000001AA, 0x87);
    cmd4(58, 0, 0xFD);

    for (auto i = 0; i < 1000; ++i) {
        cmd1(55, 0, 0x65);
        if (cmd1(41, 1<<30, 0x77) == 0)
            break;
    }

    cmd4(58, 0, 0xFD);

    logf("inited: %d ms", cycles::millis()-start);

    spiGpio.deinit();
    //sdSpi.init(SPI_PINS, 20'000);

    while (true)
        asm ("wfi");
}
