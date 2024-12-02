#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
#include <jee/i2c.h>
using namespace jeeh;
#include "defs.h"

i2c::Gpio i2cBus;
//i2c::Poll<I2C_NAME.ADDR> i2cBus (ena::I2C_NAME, I2C_FREQ);

void showPresence (char const* name, uint8_t addr) {
    i2c::Dev dev { i2cBus, addr };
    bool ack = dev.transfer(i2cBus.W1) && dev.transfer(i2cBus.W2);
    logf("%15s @ 0x%02x: %s", name, addr, ack ? "OK" : "NOT FOUND");
}

int main () {
    initBoard();
    bmpVcc = 1;

    i2cBus.init(I2C_PINS, i2cTiming(100));
    i2c::detect(i2cBus);

    showPresence("HMC5883", 0x1E);
    showPresence("128x32 OLED", 0x3C);
    showPresence("128x64 OLED", 0x3D);
    showPresence("SHT21", 0x40);
    showPresence("32 KB FRAM", 0x50);
    showPresence("ADXL345", 0x53);
    showPresence("ITG3200", 0x68);
    showPresence("BMP390", 0x77);

    while (true) {
        led = 1;
        cycles::msBusy(100);
        led = 0;
        cycles::msBusy(900);
    }
}
