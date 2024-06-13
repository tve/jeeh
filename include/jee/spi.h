namespace jeeh {

struct Blah {};

struct SpiGpio : Blah {
    Pin mosi, miso, sclk, nsel; // pin definitions must be kept in this order
    uint16_t rate =0;
    uint8_t cpol =0;

    void enable () const { nsel = 0; }
    void disable () const { nsel = 1; }

    void init (char const* desc) {
        Pin::config(desc, &mosi, 4);
        disable(); // start with NSEL high
        Pin::config(":P,:U,:P,", &mosi, 4);
        sclk = cpol;
    }

    void deinit () {
        Pin::config(":F,,,:U", &mosi, 4); // keep NSEL pulled up
    }

    int transfer (int v) const {
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

    void transfer (int32_t req, uint8_t const* cmd, uint8_t* buf) const {
        auto nCmd = (req >> 16) & 0x0F;
        for (auto i = 0; i < nCmd; ++i)
            transfer(cmd[i]);
        auto nBuf = (uint16_t) req;
        if (req < 0)
            for (auto i = 0U; i < nBuf; ++i)
                transfer(buf[i]);
        else
            for (auto i = 0U; i < nBuf; ++i)
                buf[i] = transfer(0);
    }

private:
    void hold () const {
        for (volatile int i = rate; --i >= 0; ) {}
    }
};

struct SpiBase {
    virtual void enable () =0;
    virtual void disable () =0;
    virtual int transfer (int v) =0;
    virtual void transfer (int32_t req, uint8_t const* cmd, uint8_t* buf) =0;
};

template< typename SPI >
struct SpiWrap final : SpiBase, SPI {
    void enable () override { SPI::enable(); }
    void disable () override { SPI::disable(); }
    int transfer (int v) override { return SPI::transfer(v); }
    void transfer (int32_t req, uint8_t const* cmd, uint8_t* buf) override {
        SPI::transfer(req, cmd, buf);
    }
};

} // namespace jeeh
