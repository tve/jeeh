// DMA-based I2C tests.

#include "common.h"
Pin pins [8];
#include "jee/ticker.h"
#include "jee/i2c.h"
#include "defs.h"

constexpr auto SPEED = 400; // I2C bus speed, kHz
constexpr auto DUMP = false; // false compares against expected

Ticker ticker;
TICKER_INSTALL(ticker)

i2c::Gpio i2cGpio;
i2c::Poll<I2C_NAME.ADDR> i2cPoll (ena::I2C_NAME, I2C_FREQ);
i2c::Sync<I2C_TYPE> i2cSync (I2C_CONF);

i2c::Work<I2C_TYPE> i2cWork (I2C_CONF);
IRQ_HANDLER(DMA1_Channel5, i2cWork.interrupt) // not DMA1_CH5 !
IRQ_HANDLER(DMA1_Channel6, i2cWork.interrupt) // not DMA1_CH6 !

template< typename T >
bool read32b (T const& dev, uint16_t addr, void* ptr) {
    addr = (addr<<8) | (addr>>8); // big-endian
    return dev.read16(addr, ptr, 32);
}

template< typename T >
bool write32b (T const& dev, uint16_t addr, void const* ptr) {
    addr = (addr<<8) | (addr>>8); // big-endian
    return dev.write16(addr, ptr, 32);
}

void setUp () {}

void tearDown () {
    i2cGpio.deinit();
    i2cPoll.deinit();
    i2cSync.deinit();
    i2cWork.deinit();
}

void testFramGpio () {
    if (DUMP)
        logf("\n<<< testFramGpio >>>");
    i2cGpio.init(I2C_PINS, SPEED);
    i2c::Dev fram { i2cGpio, 0x50 };

    //i2c::detect(i2cGpio);

    auto detect = [&](uint8_t addr) {
        i2c::Dev dev { i2cGpio, addr };
        return dev.transfer(i2cGpio.W1) && dev.transfer(i2cGpio.W2);
    };

    TEST_ASSERT_FALSE(detect(0x4F));
    TEST_ASSERT_TRUE(detect(0x50));
    //TEST_ASSERT_FALSE(detect(0x51));

    // read FRAM's device ID, MB85RC256V.pdf p10
    i2cGpio.start(0xF8);
    i2cGpio.wrByte(fram.id<<1);
    i2cGpio.start(0xF9);
    auto id = i2cGpio.rdByte(false) << 16;
    id |= i2cGpio.rdByte(false) << 8;
    id |= i2cGpio.rdByte(true);
    TEST_ASSERT_EQUAL_HEX(0x00A510, id);

    uint8_t buf [32], buf2 [32];

    memset(buf, 0xEE, sizeof buf);
    for (auto i = 0; i < 3; ++i)
        write32b(fram, 32*i, buf);

    for (auto i = 0; i < 3; ++i) {
        memset(buf2, 0x55, sizeof buf2);
        read32b(fram, 32*i, buf2);
        if (DUMP)
            logDump(buf2, 16, "0xEE");
        else
            TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf);
    }

    for (auto i = 0; i < 3; ++i) {
        memset(buf, i+0x80, sizeof buf);
        write32b(fram, 32*i, buf);
    }

    for (auto i = 0; i < 3; ++i) {
        memset(buf, i+0x80, sizeof buf);
        memset(buf2, 0xAA, sizeof buf2);
        read32b(fram, 32*i, buf2);
        if (DUMP)
            logDump(buf2, 16, "0x80, 0x81, ...");
        else
            TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf);
    }
}

void testFramPoll () {
    if (DUMP)
        logf("\n<<< testFramPoll >>>");
    i2cPoll.init(I2C_PINS, i2cTiming(SPEED));
    //i2cPoll.init(I2C_PINS, 0x20B03844);
    i2c::Dev fram { i2cPoll, 0x50 };

    uint8_t buf [32], buf2 [32];

    memset(buf, 0xDD, sizeof buf);
    for (auto i = 0; i < 3; ++i) {
        auto f = write32b(fram, 32*i, buf);
        TEST_ASSERT(f);
    }

    for (auto i = 0; i < 3; ++i) {
        memset(buf2, 0x55, sizeof buf2);
        auto f = read32b(fram, 32*i, buf2);
        TEST_ASSERT(f);
        if (DUMP)
            logDump(buf2, 16, "0xDD");
        else
            TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf);
    }

    for (auto i = 0; i < 3; ++i) {
        memset(buf, i+0x90, sizeof buf);
        auto f = write32b(fram, 32*i, buf);
        TEST_ASSERT(f);
    }

    for (auto i = 0; i < 3; ++i) {
        memset(buf, i+0x90, sizeof buf);
        memset(buf2, 0xAA, sizeof buf2);
        auto f = read32b(fram, 32*i, buf2);
        TEST_ASSERT(f);
        if (DUMP)
            logDump(buf2, 16, "0x90, 0x91, ...");
        else
            TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf);
    }
}

void testFramSync () {
    if (DUMP)
        logf("\n<<< testFramSync >>>");
    i2cSync.init(I2C_PINS, i2cTiming(SPEED));
    i2c::Dev fram { i2cSync, 0x50 };

    uint8_t buf [32], buf2 [32];

    memset(buf, 0xCC, sizeof buf);
    for (auto i = 0; i < 3; ++i)
        write32b(fram, 32*i, buf);

    for (auto i = 0; i < 3; ++i) {
        memset(buf2, 0x55, sizeof buf2);
        read32b(fram, 32*i, buf2);
        if (DUMP)
            logDump(buf2, 16, "0xCC");
        else
            TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf);
    }

    for (auto i = 0; i < 3; ++i) {
        memset(buf, i+0xA0, sizeof buf);
        write32b(fram, 32*i, buf);
    }

    for (auto i = 0; i < 3; ++i) {
        memset(buf, i+0xA0, sizeof buf);
        memset(buf2, 0xAA, sizeof buf2);
        read32b(fram, 32*i, buf2);
        if (DUMP)
            logDump(buf2, 16, "0xA0, 0xA1, ...");
        else
            TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf);
    }
}

void testFramWait () {
    if (DUMP)
        logf("\n<<< testFramWait >>>");
    i2cWork.init(I2C_PINS, i2cTiming(SPEED));
    i2c::Dev fram { i2cWork, 0x50 };

    uint8_t buf [32], buf2 [32];

    memset(buf, 0xEE, sizeof buf);
    for (auto i = 0; i < 3; ++i)
        write32b(fram, 32*i, buf);

    for (auto i = 0; i < 3; ++i) {
        memset(buf2, 0x55, sizeof buf2);
        read32b(fram, 32*i, buf2);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf);
    }

    for (auto i = 0; i < 3; ++i) {
        memset(buf, i+0xA0, sizeof buf);
        write32b(fram, 32*i, buf);
    }

    for (auto i = 0; i < 3; ++i) {
        memset(buf, i+128, sizeof buf);
        memset(buf2, 0xAA, sizeof buf2);
        read32b(fram, 32*i, buf2);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf);
    }
}

#if 0
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
    if (DUMP)
        logf("\n<<< testFramWork >>>");
    I2cWorker worker;
    auto swId = i2cWork.init(I2C_PINS, i2cTiming(SPEED));
    auto wkId = worker.init();

    TEST_ASSERT_GREATER_THAN(0, wkId);
    TEST_ASSERT_GREATER_THAN(wkId, swId);

    // TODO FRAM driver will need to be extended to work in async mode
    //  i.e. wrap as worker and use periodic ticks to check erase completion
}
#endif

void allTests () {
    Pin::config("A15:U,B7,B5:P,B4,A11,B3,A1,A0", pins, sizeof pins);
    for (auto e : pins) e = 0;
    pins[4] = 1; // NSEL
pins[4] = 0; // ~NSEL
pins[4] = 1; // NSEL

    //RUN_TEST(testFramGpio);
pins[6] = 1;
    RUN_TEST(testFramPoll);
pins[6] = 0;
    RUN_TEST(testFramSync);
    //RUN_TEST(testFramWait); // async in blocking mode (sync-like)
    //RUN_TEST(testFramWork);
logf("99");
}
