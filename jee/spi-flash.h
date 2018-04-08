#include <jee.h>

// driver for WinBond W25Qxx spi flash memory

template< typename SPI >
class SpiFlash {
    static void cmd (int arg) {
        SPI::enable();
        SPI::transfer(arg);
    }
    static void wait () {
        SPI::disable();
        cmd(0x05);
        while (SPI::transfer(0) & 1)
            ;
        SPI::disable();
    }
    static void wcmd (int arg) {
        wait();
        cmd(0x06);
        SPI::disable();
        cmd(arg);
    }
    static void w24b (int page) {
        SPI::transfer(page >> 8);
        SPI::transfer(page);
        SPI::transfer(0);
    }

public:
    static void init () {}

    static int devId () {
        cmd(0x9F);
        int r = SPI::transfer(0) << 16;
        r |= SPI::transfer(0) << 8;
        r |= SPI::transfer(0);
        SPI::disable();
        return r;
    }

    static int size () {
        // works for WinBond W25Qxx, e.g. W25Q64 => 0xC84017 => 8192 KB
        return 1 << ((devId() & 0xFF) - 10);
    }

    static void wipe () {
        wcmd(0x60);
        wait();
    }

    static void erase (int page) {
        wcmd(0x20);
        w24b(page);
        wait();
    }

    static void read256 (int page, void* buf) {
        cmd(0x03);
        w24b(page);
        for (int i = 0; i < 256; ++i)
            ((uint8_t*) buf)[i] = SPI::transfer(0);
        SPI::disable();
    }

    static void write256 (int page, const void* buf) {
        wcmd(0x02);
        w24b(page);
        for (int i = 0; i < 256; ++i)
            SPI::transfer(((uint8_t*) buf)[i]);
        wait();
    }
};
