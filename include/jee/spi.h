namespace jeeh::spi {

template< typename SPI >
struct Dev {
    SPI& bus;

    Dev (SPI& b) : bus (b) {}

    bool transfer (bool w, void* p =nullptr, uint8_t n =0) const {
        return bus.transfer(w, (uint8_t*) p, n);
    }

    // one byte address, single-byte data
    int32_t read (uint8_t r) const {
        uint32_t v = 0;
        return read(r, &v, 1) ? v : -1;
        return v;
    }
    bool write (uint8_t r, uint8_t v) const {
        return write(r, &v, 1);
    }

    // one byte address, to/from buffer
    bool read (uint8_t r, void* p, uint8_t n) const {
        bus.enable();
        transfer(true, &r, 1);
        transfer(false, p, n);
        bus.disable();
        return true;
    }
    bool write (uint8_t r, void const* p, uint8_t n) const {
        bus.enable();
        transfer(true, &r, 1);
        transfer(true, (void*) p, n);
        bus.disable();
        return true;
    }

    // two byte address, to/from buffer
    bool read16 (uint16_t r, void* p, uint8_t n) const {
        bus.enable();
        transfer(true, &r, 2);
        transfer(true, p, n);
        bus.disable();
        return true;
    }
    bool write16 (uint16_t r, void const* p, uint8_t n) const {
        bus.enable();
        transfer(true, &r, 2);
        transfer(true, (void*) p, n);
        bus.disable();
        return true;
    }
};

struct Gpio {
    Pin mosi, miso, sclk, nsel; // pin definitions must be kept in this order
    uint16_t rate =0;
    uint8_t cpol =0;

    void init (char const* desc, int khz =10'000) {
        Pin::config(desc, &mosi, 4);
        disable(); // start with NSEL high
        Pin::config(":HP,:D,:HP,", &mosi, 4);
        sclk = cpol;

        rate = khz < 1000 ? khz : SystemCoreClock/khz/200'000; // TODO
    }

    void deinit () {
        Pin::config(":F,,,", &mosi, 4);
    }

    void enable () const { hold(); nsel = 0; hold(); }
    void disable () const { hold(); nsel = 1; hold(); }

    int rwByte (int v) const {
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

    uint8_t transfer (uint8_t w, uint8_t* p, uint16_t n) const {
        uint8_t r = 0;
        auto q = (uint8_t*) p;
        if (w)
            for (auto i = 0U; i < n; ++i)
                r = rwByte(*q++); // return last byte from reply
        else
            for (auto i = 0U; i < n; ++i)
                *q++ = rwByte(0);
        return r;
    }

private:
    void hold () const {
        for (volatile int i = rate; --i >= 0; ) {}
    }
};

struct Base {
    virtual void enable () =0;
    virtual void disable () =0;
    virtual int rwByte (int v) =0;
    virtual uint8_t transfer (uint8_t m, uint8_t* p, uint16_t n) const =0;
};

template< typename SPI >
struct Wrap final : Base, SPI {
    void enable () override { SPI::enable(); }
    void disable () override { SPI::disable(); }
    int rwByte (int v) override { return SPI::rwByte(v); }
    uint8_t transfer (uint8_t m, uint8_t* p, uint16_t n) const override {
        return SPI::transfer(m, p, n);
    }
};

} // namespace jeeh
