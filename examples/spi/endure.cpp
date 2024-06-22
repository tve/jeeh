// SPI Flash endurance test.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard("endure");

    //SpiGpio spi;
    SpiPoll<SPI_NAME.ADDR> spi (ena::SPI_NAME, SPI_FREQ);
    //SpiSync<SPI_TYPE> spi (SPI_CONF);
    //SpiCall<SPI_TYPE> spi (SPI_CONF);

    spi.init(SPI_PINS, 85'000);
    SpiFlash spif (spi);

    constexpr auto CHIP  = 512*1024;
    constexpr auto BLOCK = 4*1024;
    constexpr auto PAGE  = 256;

    constexpr auto BPC  = CHIP/BLOCK; // blocks per chip
    constexpr auto PPB  = BLOCK/PAGE; // pages per block
    assert(BPC == 128);
    assert(PPB == 16);

    logf("\nFlash: %d kB, %d blocks of %d kB", CHIP>>10, BPC, BLOCK>>10);

    cycles::clear();
    spif.wipe();
    logf("  %d kB wiped   in %6d ms", CHIP>>10, cycles::millis());

    crc::init();
    for (auto i = 0; i < BLOCK; ++i)
        crc::update8(0xFF);
    auto crcEmpty = crc::get();
    logf("empty CRC = %08x", crcEmpty);

    uint8_t buf [PAGE] alignas(4);
    int seq = 0;

    while (true) {
        logf("\nRound #%d:", ++seq);

        rng::Permutation<BPC> bPerm;
        uint32_t bSums [BPC];

        // erase blocks in random order
        cycles::clear();
        bPerm.init();
        for (auto i = 0; i < BPC; ++i) {
            auto b = bPerm.next();
            printf("  block %5d\r", b);
            spif.erase(b * PPB);
            bSums[b] = crcEmpty;
        }
        logf("  %d kB erased  in %6d ms", CHIP>>10, cycles::millis());

        // verify that all blocks are empty in sequential order
        cycles::clear();
        for (auto b = 0; b < BPC; ++b) {
            printf("  block %5d\r", b);
            crc::init();
            for (auto p = 0; p < PPB; ++p) {
                buf[PAGE-1] = 0;
                spif.read256(b * PPB + p, buf);
                for (auto i = 0; i < PAGE; ++i)
                    crc::update8(buf[i]);
            }
            assert(crc::get() == bSums[b]);
        }
        logf("  %d kB empty   in %6d ms", CHIP>>10, cycles::millis());

        // fill all blocks with random data in random order
        cycles::clear();
        bPerm.init();
        for (auto i = 0; i < BPC; ++i) {
            auto b = bPerm.next();
            printf("  block %5d\r", b);
            crc::init();
            for (auto p = 0; p < PPB; ++p) {
                auto wp = (uint32_t*) buf;
                auto wn = PAGE/4;
                for (auto i = 0; i < wn; ++i)
                    wp[i] = rng::rand();
                for (auto i = 0; i < PAGE; ++i)
                    crc::update8(buf[i]);
                spif.write256(b * PPB + p, buf);
            }
            bSums[b] = crc::get();
        }
        logf("  %d kB written in %6d ms", CHIP>>10, cycles::millis());

        // verify that all block checksums match in sequential order
        cycles::clear();
        for (auto b = 0; b < BPC; ++b) {
            printf("  block %5d\r", b);
            crc::init();
            for (auto p = 0; p < PPB; ++p) {
                buf[PAGE-1] = 0;
                spif.read256(b * PPB + p, buf);
                for (auto i = 0; i < PAGE; ++i)
                    crc::update8(buf[i]);
            }
            assert(crc::get() == bSums[b]);
        }
        logf("  %d kB matched in %6d ms", CHIP>>10, cycles::millis());
        logDump(bSums, 64, "bSums");
    }
}
