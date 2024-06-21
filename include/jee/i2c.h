namespace jeeh {

struct I2cGpio {
    using ID = uint8_t;

    Pin sda, scl; // pin definitions must be kept in this order
    uint16_t rate;

    void init (char const* desc, uint16_t khz =400);

    void deinit () {
        Pin::config(":F,:U", &sda, 2); // keep SCL pulled up
    }

    void detect () const;

    enum { R1, R2, W1, W2 };

    bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        bool ack = true;

        if (m == R1 || m == W1)
            ack = start(2*a);

        if (ack) {
            auto q = (uint8_t*) p;
            if (m != R2) {
                for (auto i = 0; ack && i < n; ++i)
                    ack = wrByte(*q++);
            } else {
                ack = start(2*a + 1);
                for (auto i = 0; i < n; ++i)
                    *q++ = rdByte(i == n-1);
            }
        }

        if (m == R2 || m == W2 || !ack)
            stop();

        return ack;
    }

    bool start (uint8_t addr) const;
    void stop () const;

    int rdByte (bool last) const;
    bool wrByte (uint8_t data) const;

private:
    void hold () const {
        for (volatile int i = rate; --i >= 0; ) {}
    }
    void sclLo () const {
        hold();
        scl = 0;
    }
    void sclHi () const {
        hold();
        scl = 1;
        for (auto i = 10'000; scl == 0 && i >= 0; --i) {}
    }
};

struct I2cBase {
    using ID = uint8_t;

    virtual bool start (uint8_t addr) =0;
    virtual void stop () =0;
    virtual int rdByte (bool last) =0;
    virtual bool wrByte (uint8_t data) =0;
    virtual bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const =0;
};

template< typename I2C >
struct I2cWrap final : I2cBase, I2C {
    bool start (uint8_t a) override { return I2C::start(a); }
    void stop () override { I2C::stop(); }
    int rdByte (bool l) override { return I2C::rdByte(l); }
    bool wrByte (uint8_t d) override { return I2C::wrByte(d); }
    bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const override {
        return I2C::transfer(a, m, p, n);
    }
};

} // namespace jeeh
