namespace jeeh {

struct SpiGpio {
    using ID = Pin;

    Pin mosi, miso, sclk, nsel; // pin definitions must be kept in this order
    uint16_t rate =0;
    uint8_t cpol =0;
    BusDev<SpiGpio> dev {*this, Pin{}};

    enum { R1, W1, R2, W2 };

    void init (char const* desc, int khz =10'000) {
        Pin::config(desc, &mosi, 4);
        disable(); // start with NSEL high
        Pin::config(":P,:U,:P,", &mosi, 4);
        sclk = cpol;

        rate = khz < 1000 ? khz : SystemCoreClock/khz/200'000; // TODO
    }

    void deinit () {
        Pin::config(":F,,,:U", &mosi, 4); // keep NSEL pulled up
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

    // cmd = pfxLen byte + prefix data, buf & len = bytes to read or write
    // write buf if pfxLen bit 7 is set, else read
    uint8_t rwCmd (void const* cmd, uint8_t* buf =0, uint16_t len =0) {
        auto p = (uint8_t const*) cmd;
        int8_t n = *p++;
        auto r = transfer(n < 0 ? W1 : R1, (uint8_t*) p, n & 0x7F);
        transfer(n < 0 ? W2 : R2, buf, len);
        return r;
    }

    uint8_t transfer (uint8_t m, uint8_t* p, uint16_t n) const {
        uint8_t r = 0;
        if (m <= W1)
            enable();

        auto q = (uint8_t*) p;
        if (m != R2)
            for (auto i = 0U; i < n; ++i)
                r = rwByte(*q++); // return last byte from reply
        else
            for (auto i = 0U; i < n; ++i)
                *q++ = rwByte(0);

        if (m >= R2)
            disable();
        return r;
    }

private:
    void hold () const {
        for (volatile int i = rate; --i >= 0; ) {}
    }
};

struct SpiBase {
    virtual void enable () =0;
    virtual void disable () =0;
    virtual int rwByte (int v) =0;
    virtual uint8_t rwCmd (void const* c, uint8_t* p =0, uint16_t n =0) =0;
    virtual uint8_t transfer (uint8_t m, uint8_t* p, uint16_t n) const =0;
};

template< typename SPI >
struct SpiWrap final : SpiBase, SPI {
    void enable () override { SPI::enable(); }
    void disable () override { SPI::disable(); }
    int rwByte (int v) override { return SPI::rwByte(v); }
    uint8_t rwCmd (void const* c, uint8_t* p =0, uint16_t n =0) override {
        return SPI::rwCmd(c, p, n);
    }
    uint8_t transfer (uint8_t m, uint8_t* p, uint16_t n) const override {
        return SPI::transfer(m, p, n);
    }
};

} // namespace jeeh
