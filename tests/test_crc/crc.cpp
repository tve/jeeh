// Cyclic Redundancy Check tests.

#include "../common.h"
#include "jee/crc.h"

void setUp () {}
void tearDown () {}

void testCrc () {
    crc::init();
    TEST_ASSERT_EQUAL_HEX(0xFFFF'FFFF, crc::get());

    auto start = cycles::micros();
    crc::init();
    for (auto i = 0; i < 256; ++i)
        crc::update8(i);
    TEST_ASSERT_EQUAL(10, cycles::micros()-start);
    TEST_ASSERT_EQUAL_HEX(0x494A'116A, crc::get());

    start = cycles::micros();
    crc::init();
    for (auto i = 0; i < 256; ++i)
        crc::update16(i);
    TEST_ASSERT_EQUAL(10, cycles::micros()-start);
    TEST_ASSERT_EQUAL_HEX(0xD5B0'BA1C, crc::get());

    start = cycles::micros();
    crc::init();
    for (auto i = 0; i < 256; ++i)
        crc::update32(i);
    TEST_ASSERT_EQUAL(9, cycles::micros()-start);
    TEST_ASSERT_EQUAL_HEX(0x9667'0628, crc::get());
}

void allTests () {
    RUN_TEST(testCrc);
}
