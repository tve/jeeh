// Simple I2C bus scan.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#if USE_GPIO
I2cGpio i2c;
#elif USE_POLL
I2cPoll<I2C_NAME.ADDR> i2c (ena::I2C_NAME, I2C_FREQ);
#elif USE_SYNC
I2cSync<I2C_TYPE> i2c (I2C_CONF);
#elif USE_CALL
I2cCall<I2C_TYPE> i2c (I2C_CONF);
#else
#error "no USE_<TYPE> defined"
#endif

BusDev fram { i2c, 0x50 };

void read32 (uint16_t addr, void* ptr) {
    addr = (addr<<8) | (addr>>8); // big-endian
    fram.read(addr, ptr, 32);
}

void write32 (uint16_t addr, void const* ptr) {
    addr = (addr<<8) | (addr>>8); // big-endian
    fram.write(addr, ptr, 32);
}

int main () {
    initBoard("fram");

    i2c.init(I2C_PINS, 1000);

#if USE_GPIO
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
