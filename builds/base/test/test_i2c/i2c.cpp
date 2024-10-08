// DMA-based I2C tests.

#include "common.h"
#include "jee/ticker.h"
#include "jee/i2c.h"
#include "defs.h"

constexpr auto MARGIN = 10000; // non-zero loosens microsecond timing checks

Ticker ticker;
TICKER_INSTALL(ticker)

i2c::Gpio i2cGpio;
i2c::Poll<I2C_NAME.ADDR> i2cPoll (ena::I2C_NAME, I2C_FREQ);
#if 0
i2c::Sync<I2C_TYPE> i2cSync (I2C_CONF);

i2c::Work<I2C_TYPE> i2cWork (I2C_CONF);
IRQ_HANDLER(DMA1_Channel3, i2cWork.interrupt) // not DMA1_CH3 !
IRQ_HANDLER(DMA1_Channel4, i2cWork.interrupt) // not DMA1_CH4 !
#endif

template< typename T >
void read32 (T const& dev, uint16_t addr, void* ptr) {
    addr = (addr<<8) | (addr>>8); // big-endian
    dev.read16(addr, ptr, 32);
}

template< typename T >
void write32 (T const& dev, uint16_t addr, void const* ptr) {
    addr = (addr<<8) | (addr>>8); // big-endian
    dev.write16(addr, ptr, 32);
}

void setUp () {}
void tearDown () {
    i2cGpio.deinit();
    i2cPoll.deinit();
    //i2cSync.deinit();
    //i2cWork.deinit();
}

void testFramGpio () {
    i2cGpio.init(I2C_PINS, 1000);
    i2c::Dev fram { i2cGpio, 0x50 };

    i2c::detect(i2cGpio);

    // read FRAM's device ID, MB85RC256V.pdf p10
    i2cGpio.start(0xF8);
    i2cGpio.wrByte(fram.id<<1);
    i2cGpio.start(0xF9);
    auto x = i2cGpio.rdByte(false);
    auto y = i2cGpio.rdByte(false);
    auto z = i2cGpio.rdByte(true);
    logf("id: %02x %02x %02x", x, y, z); // should be: 00 A5 10

    uint16_t buf [32];
    memset(buf, 0xEE, sizeof buf);
    for (auto i = 0; i < 3; ++i)
        write32(fram, 32*i, buf);

    for (auto i = 0; i < 3; ++i) {
        memset(buf, 0x55, sizeof buf);
        read32(fram, 32*i, buf);
        logDump(buf, 16);
    }

    for (auto i = 0; i < 3; ++i) {
        memset(buf, i+128, sizeof buf);
        write32(fram, 32*i, buf);
    }

    for (auto i = 0; i < 3; ++i) {
        memset(buf, 0xAA, sizeof buf);
        read32(fram, 32*i, buf);
        logDump(buf, 16);
    }
}

#if 0
void testFramPoll () {
    i2cPoll.init(I2C_PINS, i2cTiming(1000));
    i2c::Dev fram { i2cPoll, 0x50 };

    // read FRAM's device ID, MB85RC256V.pdf p10
    i2cPoll.start(0xF8);
    i2cPoll.wrByte(fram.id<<1);
    i2cPoll.start(0xF9);
    auto x = i2cPoll.rdByte(false);
    auto y = i2cPoll.rdByte(false);
    auto z = i2cPoll.rdByte(true);
    logf("id: %02x %02x %02x", x, y, z); // should be: 00 A5 10

    uint16_t buf [32];
    memset(buf, 0xEE, sizeof buf);
    for (auto i = 0; i < 3; ++i)
        write32(fram, 32*i, buf);

    for (auto i = 0; i < 3; ++i) {
        memset(buf, 0x55, sizeof buf);
        read32(fram, 32*i, buf);
        logDump(buf, 16);
    }

    for (auto i = 0; i < 3; ++i) {
        memset(buf, i+128, sizeof buf);
        write32(fram, 32*i, buf);
    }

    for (auto i = 0; i < 3; ++i) {
        memset(buf, 0xAA, sizeof buf);
        read32(fram, 32*i, buf);
        logDump(buf, 16);
    }
}

void testFramSync () {
    i2cSync.init(I2C_PINS, i2cTiming(1000));
    I2cFram fram (i2cSync);

    // expect a W25Q16 chip of 2 MB, serial# 0xE66764A5535C7323

    TEST_ASSERT_EQUAL_HEX(0xEF4015, fram.info());
    TEST_ASSERT_EQUAL(2048, fram.size());

    uint8_t snBuf [8];
    fram.serNum(snBuf);
    const uint8_t expect [] = { 0xE6,0x67,0x64,0xA5,0x53,0x5C,0x73,0x23 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, snBuf, sizeof snBuf);

    auto start = cycles::millis();
    fram.erase(0);
    TEST_ASSERT_INT_WITHIN(MARGIN, 28, cycles::millis()-start);

    uint8_t buf [512], buf2 [512];
    memset(buf, 0x55, sizeof buf);

    start = cycles::millis();
    fram.write(0, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(MARGIN, 1, cycles::millis()-start);

    fram.read(0, buf2, sizeof buf2);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf2);
}

void testFramWait () {
    i2cWork.init(I2C_PINS, i2cTiming(1000));
    i2c::Dev fram { i2cWork, 0x50 };

    // expect a W25Q16 chip of 2 MB, serial# 0xE66764A5535C7323

    TEST_ASSERT_EQUAL_HEX(0xEF4015, fram.info());
    TEST_ASSERT_EQUAL(2048, fram.size());

    uint8_t snBuf [8];
    fram.serNum(snBuf);
    const uint8_t expect [] = { 0xE6,0x67,0x64,0xA5,0x53,0x5C,0x73,0x23 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, snBuf, sizeof snBuf);

    auto start = cycles::millis();
    fram.erase(0);
    TEST_ASSERT_INT_WITHIN(MARGIN, 28, cycles::millis()-start);

    uint8_t buf [512], buf2 [512];
    memset(buf, 0x55, sizeof buf);

    start = cycles::millis();
    fram.write(0, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(MARGIN, 1, cycles::millis()-start);

    fram.read(0, buf2, sizeof buf2);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf2);
}

struct I2cWorker : Worker {
    enum TAG { TX, TX1, TX2, TX3, RX, DONE };

    uint8_t calls =0;
    bool done =false;
    uint8_t buf [100];

    using Worker::init;

private:
    Event process (Event in, Event out, void*) override {
        ++calls;

        switch (in.eTag) {
            case TX:
                i2cWork.start(true, (uint8_t*) "x", 1, { wId, TX1 });
                break;
            case TX1:
                i2cWork.start(true, (uint8_t*) "abcde", 5, { wId, TX2 });
                break;
            case TX2:
                i2cWork.start(true, (uint8_t*) "1234567890", 10, { wId, TX3 });
                break;
            case TX3:
                i2cWork.start(true,
                              (uint8_t*) "123456789012345678901234567890", 30,
                              { wId, DONE });
                break;
            case RX:
                memset(buf, 0, sizeof buf);
                i2cWork.start(false, buf, sizeof buf, { wId, DONE });
                break;
            case DONE:
                done = true;
                break;
            default:
                fail();
        }
        return out;
    }
};

void testFramWork () {
    I2cWorker worker;
    auto swId = i2cWork.init(I2C_PINS, i2cTiming(1000));
    auto wkId = worker.init();

    TEST_ASSERT_GREATER_THAN(0, wkId);
    TEST_ASSERT_GREATER_THAN(wkId, swId);

    // TODO FRAM driver will need to be extended to work in async mode
    //  i.e. wrap as worker and use periodic ticks to check erase completion

    i2c::Dev fram { i2cWork, 0x50 };

    // expect a W25Q16 chip of 2 MB, serial# 0xE66764A5535C7323

    TEST_ASSERT_EQUAL_HEX(0xEF4015, fram.info());
    TEST_ASSERT_EQUAL(2048, fram.size());

    uint8_t snBuf [8];
    fram.serNum(snBuf);
    const uint8_t expect [] = { 0xE6,0x67,0x64,0xA5,0x53,0x5C,0x73,0x23 };
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, snBuf, sizeof snBuf);

    auto start = cycles::millis();
    fram.erase(0);
    TEST_ASSERT_INT_WITHIN(MARGIN, 28, cycles::millis()-start);

    uint8_t buf [512], buf2 [512];
    memset(buf, 0x55, sizeof buf);

    start = cycles::millis();
    fram.write(0, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(MARGIN, 1, cycles::millis()-start);

    fram.read(0, buf2, sizeof buf2);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf2);
}
#endif

void allTests () {
    RUN_TEST(testFramGpio);
    //RUN_TEST(testFramPoll);
    //RUN_TEST(testFramSync);
    //RUN_TEST(testFramWait); // async in blocking mode (sync-like)
    //RUN_TEST(testFramWork);
}
