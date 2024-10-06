// DMA-based SPI tests.

#include "common.h"
#include "jee/ticker.h"
#include "jee/spi.h"
#include <jee/dev/flash.h>
#include "defs.h"

Ticker ticker;
TICKER_INSTALL(ticker)

spi::Gpio spiGpio;
spi::Poll<SPI_NAME.ADDR> spiPoll (ena::SPI_NAME, SPI_FREQ);
spi::Sync<SPI_TYPE> spiSync (SPI_CONF);

spi::Work<SPI_TYPE> spiWork (SPI_CONF);
IRQ_HANDLER(DMA1_Channel3, spiWork.interrupt) // not DMA1_CH3 !
IRQ_HANDLER(DMA1_Channel4, spiWork.interrupt) // not DMA1_CH4 !

void setUp () {}
void tearDown () {
    spiGpio.deinit();
    spiPoll.deinit();
    spiSync.deinit();
}

void testTxGpio () {
    spiGpio.init(SPI_PINS, 80'000);

    auto start = cycles::micros();
    spiGpio.transfer(true, (uint8_t*) "x", 1);
    TEST_ASSERT_INT_WITHIN(1, 6, cycles::micros()-start);

    start = cycles::micros();
    spiGpio.transfer(true, (uint8_t*) "abcde", 5);
    TEST_ASSERT_INT_WITHIN(1, 29, cycles::micros()-start);

    start = cycles::micros();
    spiGpio.transfer(true, (uint8_t*) "1234567890", 10);
    TEST_ASSERT_INT_WITHIN(2, 57, cycles::micros()-start);

    start = cycles::micros();
    spiGpio.transfer(true, (uint8_t*) "123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(4, 170, cycles::micros()-start);
}

void testRxGpio () {
    spiGpio.init(SPI_PINS, 80'000);

    uint8_t buf [100];
    auto start = cycles::micros();
    spiGpio.transfer(false, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(1, 577, cycles::micros()-start);
}

void testTxPoll () {
    spiPoll.init(SPI_PINS, 80'000);

    auto start = cycles::micros();
    spiPoll.transfer(true, (uint8_t*) "x", 1);
    TEST_ASSERT_INT_WITHIN(1, 1, cycles::micros()-start);

    start = cycles::micros();
    spiPoll.transfer(true, (uint8_t*) "abcde", 5);
    TEST_ASSERT_INT_WITHIN(1, 2, cycles::micros()-start);

    start = cycles::micros();
    spiPoll.transfer(true, (uint8_t*) "1234567890", 10);
    TEST_ASSERT_INT_WITHIN(1, 3, cycles::micros()-start);

    start = cycles::micros();
    spiPoll.transfer(true, (uint8_t*) "123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(1, 7, cycles::micros()-start);
}

void testRxPoll () {
    spiPoll.init(SPI_PINS, 80'000);

    uint8_t buf [100];
    auto start = cycles::micros();
    spiPoll.transfer(false, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(1, 23, cycles::micros()-start);
}

void testTxSync () {
    spiSync.init(SPI_PINS, 80'000);

    auto start = cycles::micros();
    spiSync.transfer(true, (uint8_t*) "x", 1);
    TEST_ASSERT_INT_WITHIN(1, 2, cycles::micros()-start);

    start = cycles::micros();
    spiSync.transfer(true, (uint8_t*) "abcde", 5);
    TEST_ASSERT_INT_WITHIN(1, 2, cycles::micros()-start);

    start = cycles::micros();
    spiSync.transfer(true, (uint8_t*) "1234567890", 10);
    TEST_ASSERT_INT_WITHIN(1, 3, cycles::micros()-start);

    start = cycles::micros();
    spiSync.transfer(true, (uint8_t*) "123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(1, 7, cycles::micros()-start);
}

void testRxSync () {
    spiSync.init(SPI_PINS, 80'000);

    uint8_t buf [100];
    auto start = cycles::micros();
    spiSync.transfer(false, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(2, 23, cycles::micros()-start);
}

void testTxWait () {
    spiWork.init(SPI_PINS, 80'000);

    auto start = cycles::micros();
    spiWork.transfer(true, (uint8_t*) "x", 1);
    TEST_ASSERT_INT_WITHIN(2, 8, cycles::micros()-start);

    start = cycles::micros();
    spiWork.transfer(true, (uint8_t*) "abcde", 5);
    TEST_ASSERT_INT_WITHIN(1, 3, cycles::micros()-start);

    start = cycles::micros();
    spiWork.transfer(true, (uint8_t*) "1234567890", 10);
    TEST_ASSERT_INT_WITHIN(3, 5, cycles::micros()-start);

    start = cycles::micros();
    spiWork.transfer(true, (uint8_t*) "123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(1, 12, cycles::micros()-start);
}

void testRxWait () {
    spiWork.init(SPI_PINS, 80'000);

    uint8_t buf [100];
    auto start = cycles::micros();
    spiWork.transfer(false, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(4, 27, cycles::micros()-start);
}

void testFlashGpio () {
    spiGpio.init(SPI_PINS, 80'000);
    SpiFlash spif (spiGpio);

    // expect a W25Q16 chip of 2 MB, serial# 0xE66764A5535C7323

    TEST_ASSERT_EQUAL_HEX(0xEF4015, spif.info());
    TEST_ASSERT_EQUAL(2048, spif.size());

    uint8_t snBuf [8];
    spif.serNum(snBuf);
    const uint8_t expect [] = { 0xE6,0x67,0x64,0xA5,0x53,0x5C,0x73,0x23 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, snBuf, sizeof snBuf);

    auto start = cycles::millis();
    spif.erase(0);
    TEST_ASSERT_INT_WITHIN(4, 28, cycles::millis()-start);

    uint8_t buf [512], buf2 [512];
    memset(buf, 0x55, sizeof buf);

    start = cycles::millis();
    spif.write(0, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(1, 4, cycles::millis()-start);

    spif.read(0, buf2, sizeof buf2);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf2);
}

void testFlashPoll () {
    spiPoll.init(SPI_PINS, 80'000);
    SpiFlash spif (spiPoll);

    // expect a W25Q16 chip of 2 MB, serial# 0xE66764A5535C7323

    TEST_ASSERT_EQUAL_HEX(0xEF4015, spif.info());
    TEST_ASSERT_EQUAL(2048, spif.size());

    uint8_t snBuf [8];
    spif.serNum(snBuf);
    const uint8_t expect [] = { 0xE6,0x67,0x64,0xA5,0x53,0x5C,0x73,0x23 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, snBuf, sizeof snBuf);

    auto start = cycles::millis();
    spif.erase(0);
    TEST_ASSERT_INT_WITHIN(4, 30, cycles::millis()-start);

    uint8_t buf [512], buf2 [512];
    memset(buf, 0x55, sizeof buf);

    start = cycles::millis();
    spif.write(0, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(1, 1, cycles::millis()-start);

    spif.read(0, buf2, sizeof buf2);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf2);
}

void testFlashSync () {
    spiSync.init(SPI_PINS, 80'000);
    SpiFlash spif (spiSync);

    // expect a W25Q16 chip of 2 MB, serial# 0xE66764A5535C7323

    TEST_ASSERT_EQUAL_HEX(0xEF4015, spif.info());
    TEST_ASSERT_EQUAL(2048, spif.size());

    uint8_t snBuf [8];
    spif.serNum(snBuf);
    const uint8_t expect [] = { 0xE6,0x67,0x64,0xA5,0x53,0x5C,0x73,0x23 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, snBuf, sizeof snBuf);

    auto start = cycles::millis();
    spif.erase(0);
    TEST_ASSERT_INT_WITHIN(4, 28, cycles::millis()-start);

    uint8_t buf [512], buf2 [512];
    memset(buf, 0x55, sizeof buf);

    start = cycles::millis();
    spif.write(0, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(1, 1, cycles::millis()-start);

    spif.read(0, buf2, sizeof buf2);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf2);
}

void testFlashWait () {
    spiWork.init(SPI_PINS, 80'000);
    SpiFlash spif (spiWork);

    // expect a W25Q16 chip of 2 MB, serial# 0xE66764A5535C7323

    TEST_ASSERT_EQUAL_HEX(0xEF4015, spif.info());
    TEST_ASSERT_EQUAL(2048, spif.size());

    uint8_t snBuf [8];
    spif.serNum(snBuf);
    const uint8_t expect [] = { 0xE6,0x67,0x64,0xA5,0x53,0x5C,0x73,0x23 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, snBuf, sizeof snBuf);

    auto start = cycles::millis();
    spif.erase(0);
    TEST_ASSERT_INT_WITHIN(4, 28, cycles::millis()-start);

    uint8_t buf [512], buf2 [512];
    memset(buf, 0x55, sizeof buf);

    start = cycles::millis();
    spif.write(0, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(1, 1, cycles::millis()-start);

    spif.read(0, buf2, sizeof buf2);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf2);
}

void allTests () {
    RUN_TEST(testTxGpio);
    RUN_TEST(testRxGpio);
    RUN_TEST(testTxPoll);
    RUN_TEST(testRxPoll);
    RUN_TEST(testTxSync);
    RUN_TEST(testRxSync);
    RUN_TEST(testTxWait);
    RUN_TEST(testRxWait);
    RUN_TEST(testFlashGpio);
    RUN_TEST(testFlashPoll);
    RUN_TEST(testFlashSync);
    RUN_TEST(testFlashWait); // async in blocking mode (sync-like)
}
