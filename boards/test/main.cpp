#include <jee.h>
#include <jee/cycles.h>
#include "jee/dma.h"
#include "jee/i2c.h"
#include "jee/spi.h"
#include <jee/uart.h>
using namespace jeeh;
#include "defs.h"

#if MODE_GPIO

Dev<i2c::Gpio<I2C_CONF>> i2cBus;
Dev<spi::Gpio<SPI_CONF>> spiBus;

#elif MODE_POLL

Dev<i2c::Poll<I2C_CONF>> i2cBus;
Dev<spi::Poll<SPI_CONF>> spiBus;

#elif MODE_SYNC

Dev<i2c::Sync<I2C_CONF>> i2cBus;
Dev<spi::Sync<SPI_CONF>> spiBus;

#elif MODE_ASYNC

Dev<i2c::Async<I2C_CONF>> i2cBus;
I2C_TRIGGER(i2cBus)
Dev<spi::Async<SPI_CONF>> spiBus;
SPI_TRIGGER(spiBus)

#endif

#include "t-bmp-i.cpp"
#include "t-bmp-s.cpp"
#include "t-fram.cpp"
#include "t-imu.cpp"
#include "t-lcd.cpp"
#include "t-oled.cpp"
#include "t-scan.cpp"
//#include "t-sdspi.cpp"
#include "t-sht21.cpp"
#include "t-sram.cpp"

void header (char const* text) {
    logf("%70s", text);
}

int main () {
    initBoard();

#if 1
    i2cBus.init(1000);
    header("I2C - SCAN");   testScan();
    header("I2C - OLED");   testOled();
    header("I2C - FRAM");   testFram();
    header("I2C - IMU");    testImu();
    header("I2C - SHT21");  testSht21();
    header("I2C - BMP390"); testBmpI();
    i2cBus.deinit();
#endif

#if 1
    spiBus.init(20'000);
    header("SPI - BMP390"); testBmpS();
    header("SPI - LCD");    testLcd();
    spiBus.init(20'000); // reinit wirh proper MISO pin mode
    header("SPI - SRAM");   testSram();
    //header("SPI - SDCARD"); testSdSpi();
    spiBus.deinit();
#endif

    header("DONE");
    while (true) { cycles::msBusy(500); led.toggle(); }
}
