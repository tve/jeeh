namespace jeeh {

struct SpiGpio {
    using ID = Pin;

    Pin mosi, miso, sclk, nsel; // pin definitions must be kept in this order
    uint16_t rate =0;
    uint8_t cpol =0;
    BusDev<SpiGpio> dev {*this, Pin{}};

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

    int ioByte (int v) const {
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

    enum { R1, W1, R2, W2 };

    uint8_t transfer (uint8_t m, uint8_t* p, uint16_t n) const {
        uint8_t r = 0;
        if (m <= W1)
            enable();

        auto q = (uint8_t*) p;
        if (m != R2)
            for (auto i = 0U; i < n; ++i)
                r = ioByte(*q++); // return last byte from reply
        else
            for (auto i = 0U; i < n; ++i)
                *q++ = ioByte(0);

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
    virtual int ioByte (int v) =0;
    virtual uint8_t transfer (uint8_t m, uint8_t* p, uint16_t n) const =0;
};

template< typename SPI >
struct SpiWrap final : SpiBase, SPI {
    void enable () override { SPI::enable(); }
    void disable () override { SPI::disable(); }
    int ioByte (int v) override { return SPI::ioByte(v); }
    uint8_t transfer (uint8_t m, uint8_t* p, uint16_t n) const override {
        return SPI::transfer(m, p, n);
    }
};

} // namespace jeeh
