namespace jeeh {

struct SpiGpio {
    Pin mosi, miso, sclk, nsel; // pin definitions must be kept in this order
    uint16_t rate =0;
    uint8_t cpol =0;

    void enable () const { nsel = 0; }
    void disable () const { nsel = 1; }

    void init (char const* desc) {
        Pin::config(desc, &mosi, 4);
        disable(); // start with NSEL high
        Pin::config(":PH,:UH,:PH,", &mosi, 4);
        sclk = cpol;
    }

    void deinit () {
        Pin::config(":F,,,:U", &mosi, 4); // keep NSEL pulled up
    }

    int xfer (int v) {
        auto r = 0;
        for (auto i = 0; i < 8; ++i) {
            mosi = v >> 7;
            v <<= 1;
            hold();
            sclk = ~cpol;
            hold();
            r = (r<<1) | miso;
            sclk = cpol;
        }
        return r;
    }

    int block (uint8_t const* out, uint8_t* in, int len) {
        int b = 0;
        for (auto i = 0; i < len; ++i) {
            b = xfer(out != nullptr ? out[i] : 0);
            if (in != nullptr)
                in[i] = b;
        }
        return b;
    }

private:
    void hold () const {
        for (volatile int i = rate; --i >= 0; ) {}
    }
};

struct SpiBase {
    virtual void enable () =0;
    virtual void disable () =0;
    virtual int xfer (int v) =0;
    virtual int block (uint8_t const* out, uint8_t* in, int len) =0;
};

template< typename SPI >
struct SpiWrap final : SpiBase, SPI {
    void enable () override { SPI::enable(); }
    void disable () override { SPI::disable(); }
    int xfer (int v) override { return SPI::xfer(v); }
    int block (uint8_t const* out, uint8_t* in, int len) override {
        return SPI::block(out, in, len);
    }
};

template< typename SPI >
struct SpiFlash {
    SPI& spi;

    SpiFlash (SPI& s) : spi (s) {}

    void reset () const {
        cmd(0x66);
        spi.disable();
        cmd(0x99);
        spi.disable();
    }

    int devId () const {
        cmd(0x9F);
        int r = spi.xfer(0) << 16;
        r |= spi.xfer(0) << 8;
        r |= spi.xfer(0);
        spi.disable();
        return r;
    }

    int size () const {
        // works for WinBond W25Qxx, e.g. W25Q64 => 0xC84017 => 8192 KB
        return 1 << ((devId() & 0xFF) - 10);
    }

    void wipe () const {
        wcmd(0xC7); // 0x60 doesn't work on Micron Tech (N25Q)
        wait();
    }

    void erase (int page) const {
        wcmd(0x20);
        w24b(page<<8);
        wait();
    }

    void read256 (int page, uint8_t* buf) const {
        read(page<<8, buf, 256);
    }

    void read (int offset, uint8_t* buf, int cnt) const {
        cmd(0x0B);
        w24b(offset);
        spi.xfer(0);
        spi.block(nullptr, buf, cnt);
        spi.disable();
    }

    void write256 (int page, const uint8_t* buf) const {
        write(page<<8, buf, 256);
    }

    void write (int offset, const uint8_t* buf, int cnt) const {
        wcmd(0x02);
        w24b(offset);
        spi.block(buf, nullptr, cnt);
        wait();
    }

private:
    void cmd (int arg) const {
        spi.enable();
        spi.xfer(arg);
    }
    void wait () const {
        spi.disable();
        cmd(0x05);
        while (spi.xfer(0) & 1) {}
        spi.disable();
    }
    void wcmd (int arg) const {
        wait();
        cmd(0x06);
        spi.disable();
        cmd(arg);
    }
    void w24b (int offset) const {
        spi.xfer(offset >> 16);
        spi.xfer(offset >> 8);
        spi.xfer(offset);
    }
};

} // namespace jeeh
