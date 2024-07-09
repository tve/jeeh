namespace jeeh {

// cmd = pfxLen byte + prefix data, buf & len = bytes to read or write
// write buf if pfxLen bit 7 is set, else read
template< typename SPI >
uint8_t rwCmd (SPI& spi, void const* cmd, uint8_t* buf =0, uint16_t len =0) {
    auto ptr = (uint8_t*) cmd;
    auto send = *ptr >> 7;
    int8_t nCmd = *ptr++ & 0x7F;

    spi.enable();
    auto r = nCmd > 0 ? spi.transfer(true, ptr, nCmd) : 0;
    if (len > 0)
        spi.transfer(send, buf, len);
    spi.disable();
    return r;
}

template< typename SPI >
struct SpiFlash {
    SPI& spi;

    SpiFlash (SPI& s) : spi (s) {}

    void reset () const {
        rwCmd(spi, "\x01\x66");
        rwCmd(spi, "\x01\x99");
    }

    int info () const {
        uint8_t buf [3];
        rwCmd(spi, "\x01\x9F", buf, sizeof buf);
        return (buf[0] << 16) | (buf[1] << 8) | buf[2];
    }

    int size () const {
        // works for WinBond W25Qxx, e.g. W25Q64 => 0xC84017 => 8192 KB
        return 1 << ((info() & 0xFF) - 10);
    }

    void serNum (uint8_t* buf) const {
        memset(buf, 0, 8);
        rwCmd(spi, "\x05\x4B....", buf, 8);
    }

    void wipe () const {
        unlock();
        rwCmd(spi, "\x01\xC7"); // 0x60 doesn't work on Micron Tech (N25Q)
        wait();
    }

    void erase (int page) const {
        unlock();
        rwCmd(spi, cmdAddr(0x20, page<<8));
        wait();
    }

    void read256 (int page, uint8_t* buf) const {
        read(page<<8, buf, 256);
    }

    void read (int offset, uint8_t* buf, int len) const {
        auto p = cmdAddr(0x0B, offset);
        *p += 1; // add dummy byte
        rwCmd(spi, p, buf, len);
    }

    void write256 (int page, const uint8_t* buf) const {
        write(page<<8, buf, 256);
    }

    void write (int offset, const uint8_t* buf, int len) const {
        unlock();
        auto p = cmdAddr(0x02, offset);
        *p |= 0x80; // write
        rwCmd(spi, p, (uint8_t*) buf, len);
        wait();
    }

private:
    mutable uint8_t buf [6]; // len, cmd, 3x addr, 1 spare

    void unlock () const {
        rwCmd(spi, "\x01\x06");
    }

    void wait () const {
        while (rwCmd(spi, "\x02\x05.") & 1) {}
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
