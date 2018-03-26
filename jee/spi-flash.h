#include <jee.h>

// driver for WinBond W25Qxx spi flash memory

template< typename MO, typename MI, typename CK, typename SS >
class SpiFlash {
    static void cmd (int arg) {
        spi.enable();
        spi.transfer(arg);
    }
    static void wait () {
        spi.disable();
        cmd(0x05);
        while (spi.transfer(0) & 1)
            ;
        spi.disable();
    }
    static void wcmd (int arg) {
        wait();
        cmd(0x06);
        spi.disable();
        cmd(arg);
    }
    static void w24b (int page) {
        spi.transfer(page >> 8);
        spi.transfer(page);
        spi.transfer(0);
    }

public:
    static int devId () {
        cmd(0x9F);
        int r = spi.transfer(0) << 16;
        r |= spi.transfer(0) << 8;
        r |= spi.transfer(0);
        spi.disable();
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
            ((uint8_t*) buf)[i] = spi.transfer(0);
        spi.disable();
    }

    static void write256 (int page, const void* buf) {
        wcmd(0x02);
        w24b(page);
        for (int i = 0; i < 256; ++i)
            spi.transfer(((uint8_t*) buf)[i]);
        wait();
    }

    static SpiDev<MO,MI,CK,SS> spi;
};

template< typename MO, typename MI, typename CK, typename SS >
SpiDev<MO,MI,CK,SS> SpiFlash<MO,MI,CK,SS>::spi;
