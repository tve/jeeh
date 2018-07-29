// Driver for WinBond W25Qxx spi flash memory
// see https://jeelabs.org/ref/W25Q128F.pdf

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
    static void w24b (int offset) {
        SPI::transfer(offset >> 16);
        SPI::transfer(offset >> 8);
        SPI::transfer(offset);
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

    // size returns the flash memory size in KB
    static int size () {
        // works for WinBond W25Qxx, e.g. W25Q64 => 0xC84017 => 8192 KB
        return 1 << ((devId() & 0xFF) - 10);
    }

    static void wipe () {
        wcmd(0x60);
        wait();
    }

    // erase a 4K sector, the parameter is the start address of the sector
    static void erase (int addr) {
        wcmd(0x20);
        w24b(addr);
        wait();
    }

    static void read256 (int page, void* buf) {
        read(page<<8, buf, 256);
    }

    static void read (int offset, void* buf, int cnt) {
        cmd(0x03);
        w24b(offset);
        for (int i = 0; i < cnt; ++i)
            ((uint8_t*) buf)[i] = SPI::transfer(0);
        SPI::disable();
    }

    static void write256 (int page, const void* buf) {
        write(page<<8, buf, 256);
    }

    static void write (int offset, const void* buf, int cnt) {
        wcmd(0x02);
        w24b(offset);
        for (int i = 0; i < cnt; ++i)
            SPI::transfer(((uint8_t*) buf)[i]);
        wait();
    }
};
