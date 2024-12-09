#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
#include <jee/uart.h>
#include "xdma.h"
#include "xi2c.h"
#include "xspi.h"
using namespace jeeh;
#include "defs.h"

constexpr spi::Config spiCfg {
    SPI_PINS,                               // gpio
    SPI_NAME.ADDR, ena::SPI_NAME, SPI_FREQ, // poll
    DMA1.ADDR, 1-1, 3-1, 4-1, 11, 10,       // sync
    Irq::DMA1_CH3, Irq::DMA1_CH4,           // async
};

#if MODE_GPIO

i2c::Gpio i2cBus;
Dev<spi::Gpio<spiCfg>> spiBus;

#elif MODE_POLL

i2c::Poll<I2C_NAME.ADDR> i2cBus (ena::I2C_NAME, I2C_FREQ);
Dev<spi::Poll<spiCfg>> spiBus;

#elif MODE_SYNC

i2c::Sync<I2C_TYPE> i2cBus (I2C_CONF);
Dev<spi::Sync<spiCfg>> spiBus;

#elif MODE_ASYNC

i2c::Async<I2C_TYPE> i2cBus (I2C_CONF);
I2C_TRIGGER(i2cBus)
Dev<spi::Async<spiCfg>> spiBus;
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
    logf("%79s", text);
}

int main () {
    initBoard();

#if 0
    i2cBus.init(I2C_PINS);
    header("I2C - SCAN");   testScan();
    header("I2C - OLED");   testOled();
    header("I2C - FRAM");   testFram();
    //header("I2C - IMU");    testImu();
    header("I2C - SHT21");  testSht21();
    //header("I2C - BMP390"); testBmpI();
    i2cBus.deinit();
#endif

#if 1
    spiBus.init();
    header("SPI - BMP390"); testBmpS();
    header("SPI - LCD");    testLcd();
    spiBus.init(); // reinit wirh proper MISO pin mode
    header("SPI - SRAM");   testSram();
    //header("SPI - SDCARD"); testSdSpi();
    spiBus.deinit();
#endif

    header("DONE");
    while (true) { cycles::msBusy(500); led.toggle(); }
}
