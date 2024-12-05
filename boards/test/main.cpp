#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
#include <jee/uart.h>
#include "xi2c.h"
#include "xspi.h"
using namespace jeeh;
#include "defs.h"

#if MODE_GPIO

i2c::Gpio i2cBus;
spi::Gpio spiBus;

#elif MODE_POLL

i2c::Poll<I2C_NAME.ADDR> i2cBus (ena::I2C_NAME, I2C_FREQ);
spi::Poll<SPI_NAME.ADDR> spiBus (ena::SPI_NAME, SPI_FREQ);

#elif MODE_SYNC

i2c::Sync<I2C_TYPE> i2cBus (I2C_CONF);
spi::Sync<SPI_TYPE> spiBus (SPI_CONF);

#elif MODE_ASYNC

i2c::Async<I2C_TYPE> i2cBus (I2C_CONF);
//I2C_TRIGGER(i2cBus);
spi::Async<SPI_TYPE> spiBus (SPI_CONF);
//SPI_TRIGGER(spiBus);

#endif

#include "t-bmp-i.cpp"
//#include "t-bmp-s.cpp"
#include "t-fram.cpp"
#include "t-imu.cpp"
#include "t-lcd.cpp"
#include "t-oled.cpp"
#include "t-scan.cpp"
//#include "t-sdspi.cpp"
#include "t-sht21.cpp"
#include "t-sram.cpp"

void header (char const* text) {
    logf("%79s", text);
}

int main () {
    initBoard();

#if 1
    i2cBus.init(I2C_PINS);
    //header("I2C - SCAN");   testScan();
    //header("I2C - OLED");   testOled();
    header("I2C - FRAM");   testFram();
    //header("I2C - IMU");    testImu();
    //header("I2C - SHT21");  testSht21();
    //header("I2C - BMP390"); testBmpI();
    i2cBus.deinit();
#endif

#if 0
    spiBus.init(SPI_PINS);
    //header("SPI - BMP390"); testBmpS();
    header("SPI - LCD");    testLcd();
    header("SPI - SRAM");   testSram();
    //header("SPI - SDCARD"); testSdSpi();
    spiBus.deinit();
#endif

    header("DONE");
    while (true) { cycles::msBusy(500); led.toggle(); }
}
