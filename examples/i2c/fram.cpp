// FRAM memory, connected via I2C.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

I2cDev fram { i2c, 0x50 };

void read32 (uint16_t addr, void* ptr) {
    addr = (addr<<8) | (addr>>8); // big-endian
    fram.read16(addr, ptr, 32);
}

void write32 (uint16_t addr, void const* ptr) {
    addr = (addr<<8) | (addr>>8); // big-endian
    fram.write16(addr, ptr, 32);
}

int main () {
    initBoard("fram");

    i2c.init(I2C_PINS, 1000);

#if MODE_GPIO
    i2c.detect();

    // read FRAM's device ID, MB85RC256V.pdf p10
    i2c.start(0xF8);
    i2c.wrByte(fram.id<<1);
    i2c.start(0xF9);
    auto x = i2c.rdByte(false);
    auto y = i2c.rdByte(false);
    auto z = i2c.rdByte(true);
    logf("id: %02x %02x %02x", x, y, z); // should be: 00 A5 10
#endif

    uint16_t buf [32];

    for (auto i = 0; i < 5; ++i) {
        memset(buf, i+128, sizeof buf);
        write32(32*i, buf);
    }

    for (auto i = 0; i < 5; ++i) {
        read32(32*i, buf);
        logDump(buf, 16);
    }

    while (true) { sys::wait(500); led.toggle(); }
}
