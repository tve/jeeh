namespace jeeh {

template< typename SPI >
struct SpiFlash {
    SPI& spi;

    SpiFlash (SPI& s) : spi (s) {}

    void reset () const {
        spi.rwCmd("\x01\x66");
        spi.rwCmd("\x01\x99");
    }

    int devId () const {
        uint8_t buf [3];
        spi.rwCmd("\x01\x9F", buf, sizeof buf);
        return (buf[0] << 16) | (buf[1] << 8) | buf[2];
    }

    int size () const {
        // works for WinBond W25Qxx, e.g. W25Q64 => 0xC84017 => 8192 KB
        return 1 << ((devId() & 0xFF) - 10);
    }

    void serNum (uint8_t* buf) {
        spi.rwCmd("\x05\x4B....", buf, 8);
    }

    void wipe () const {
        unlock();
        spi.rwCmd("\x01\xC7"); // 0x60 doesn't work on Micron Tech (N25Q)
        wait();
    }

    void erase (int page) const {
        unlock();
        spi.rwCmd(cmdAddr(0x20, page<<8));
        wait();
    }

    void read256 (int page, uint8_t* buf) const {
        read(page<<8, buf, 256);
    }

    void read (int offset, uint8_t* buf, int len) const {
        auto p = cmdAddr(0x0B, offset);
        *p += 1; // add dummy byte
        spi.rwCmd(p, buf, len);
    }

    void write256 (int page, const uint8_t* buf) const {
        write(page<<8, buf, 256);
    }

    void write (int offset, const uint8_t* buf, int len) const {
        unlock();
        auto p = cmdAddr(0x02, offset);
        *p |= 0x80; // write
        spi.rwCmd(p, (uint8_t*) buf, len);
        wait();
    }

private:
    mutable uint8_t buf [6]; // len, cmd, 3x addr, 1 spare

    void unlock () const {
        spi.rwCmd("\x01\x06");
    }

    void wait () const {
        while (spi.rwCmd("\x02\x05.") & 1) {}
    }

    uint8_t* cmdAddr (uint8_t cmd, uint32_t addr) const {
        buf[0] = 0x04;
        buf[1] = cmd;
        buf[2] = addr >> 16;
        buf[3] = addr >> 8;
        buf[4] = addr;
        return buf;
    }
};

} // namespace jeeh
