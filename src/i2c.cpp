#include "jee.h"
#include "jee/hal.h"

namespace jeeh {

void I2cGpio::init (char const* desc, int r) {
    Pin::config(desc, &sda, 2);
    Pin::config(":OU,", &sda, 2);
    sda = 1;
    scl = 1;
    rate = r;
}

void I2cGpio::detect () const {
    for (auto i = 0; i < 128; i += 16) {
        printf("%02x:", i);
        for (auto j = 0; j < 16; ++j) {
            int addr = i + j;
            if (0x08 <= addr && addr <= 0x77) {
                bool ack = start(2*addr);
                stop();
                printf(ack ? " %02x" : " --", addr);
            } else
                printf("   ");
        }
        printf("\n");
    }
}

bool I2cGpio::start (uint8_t addr) const {
    sclLo();
    sclHi();
    sda = 0;
    return write(addr);
}

void I2cGpio::stop () const {
    sda = 0;
    sclHi();
    sda = 1;
    hold();
}

int I2cGpio::read (bool last) const {
    uint8_t data = 0;
    for (auto mask = 0x80; mask != 0; mask >>= 1) {
        sclHi();
        if (sda)
            data |= mask;
        sclLo();
    }
    sda = last;
    sclHi();
    sclLo();
    if (last)
        stop();
    sda = 1;
    return data;
}

bool I2cGpio::write (uint8_t data) const {
    sclLo();
    for (auto mask = 0x80; mask != 0; mask >>= 1) {
        sda = (data & mask) != 0;
        sclHi();
        sclLo();
    }
    sda = 1;
    sclHi();
    hold();
    bool ack = !sda;
    sclLo();
    return ack;
}

int I2cGpio::readReg (int addr, int reg) const {
    uint8_t val;
    if (!readRegs(addr, reg, &val, sizeof val))
        return -1;
    return val;
}

bool I2cGpio::writeReg (int addr, int reg, int val) const {
    start(2*addr);
    auto ack = write(reg);
    if (ack)
        ack = write(val);
    stop();
    return ack;
}

bool I2cGpio::readRegs (int addr, int reg, void* ptr, int len) const {
    start(2*addr);
    if (!write(reg)) {
        stop();
        return false;
    }
    start(2*addr+1);
    auto buf = (uint8_t*) ptr;
    for (auto i = 0; i < len; ++i)
        buf[i] = read(i == len-1);
    return true;
}

bool I2cGpio::writeRegs (int addr, int reg, void const* ptr, int len) const {
    start(2*addr);
    bool ack = write(reg);
    auto buf = (uint8_t const*) ptr;
    for (auto i = 0; ack && i < len; ++i)
        ack = write(buf[i]);
    stop();
    return ack;
}

} // namespace jeeh
