// DMA-based SPI tests.

#include "common.h"
#include "jee/ticker.h"
//#include "jee/spi.h"
#include "defs.h"

Ticker ticker;
TICKER_INSTALL(ticker)

spi::Gpio spiGpio;
spi::Poll<SPI_NAME.ADDR> spiPoll (ena::SPI_NAME, SPI_FREQ);
spi::Sync<SPI_TYPE> spiSync (SPI_CONF);
IRQ_HANDLER(DMA1_Channel3, spiSync.interrupt) // not DMA1_CH3 !
IRQ_HANDLER(DMA1_Channel4, spiSync.interrupt) // not DMA1_CH4 !

void setUp () {}
void tearDown () {
    spiGpio.deinit();
    spiPoll.deinit();
}

void testGpio () {
    spiGpio.init(SPI_PINS, 80'000);

    auto start = cycles::micros();
    spiGpio.transfer(true, (uint8_t*) "x", 1);
    TEST_ASSERT_INT_WITHIN(1, 6, cycles::micros()-start);

    start = cycles::micros();
    spiGpio.transfer(true, (uint8_t*) "abcde", 5);
    TEST_ASSERT_INT_WITHIN(1, 29, cycles::micros()-start);

    start = cycles::micros();
    spiGpio.transfer(true, (uint8_t*) "1234567890", 10);
    TEST_ASSERT_INT_WITHIN(1, 57, cycles::micros()-start);

    start = cycles::micros();
    spiGpio.transfer(true, (uint8_t*) "123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(1, 170, cycles::micros()-start);
}

void testPoll () {
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

void testSync () {
    spiSync.init(SPI_PINS, 80'000);

    auto start = cycles::micros();
    spiSync.transfer(true, (uint8_t*) "x", 1);
    TEST_ASSERT_INT_WITHIN(1, 3, cycles::micros()-start);

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

void allTests () {
    RUN_TEST(testGpio);
    RUN_TEST(testPoll);
    RUN_TEST(testSync);
}
