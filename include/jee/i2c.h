namespace jeeh::i2c {

template< typename I2C >
struct Dev {
    I2C& bus;
    uint8_t id;

    Dev (I2C& b, uint8_t i) : bus (b), id (i) {}

    bool transfer (uint8_t m, void* p =nullptr, uint8_t n =0) const {
        return bus.transfer(id, m, p, n);
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
        return transfer(bus.R1, &r, 1)
            && transfer(bus.R2, p, n);
    }
    bool write (uint8_t r, void const* p, uint8_t n) const {
        return transfer(bus.W1, &r, 1)
            && transfer(bus.W2, (void*) p, n);
    }

    // two byte address, to/from buffer
    bool read16 (uint16_t r, void* p, uint8_t n) const {
        return transfer(bus.R1, &r, 2)
            && transfer(bus.R2, p, n);
    }
    bool write16 (uint16_t r, void const* p, uint8_t n) const {
        return transfer(bus.W1, &r, 2)
            && transfer(bus.W2, (void*) p, n);
    }
};

template< typename I2C >
void detect (I2C& bus) {
    for (auto i = 0; i < 128; i += 16) {
        printf("%02x:", i);
        for (auto j = 0; j < 16; ++j) {
            uint8_t addr = i + j;
            if (0x08 <= addr && addr <= 0x77) {
                Dev dev { bus, addr };
                bool ack = dev.transfer(bus.W1) && dev.transfer(bus.W2);
                printf(ack ? " %02x" : " --", addr);
            } else
                printf("   ");
        }
        printf("\n");
    }
}

struct Gpio {
    using ID = uint8_t;

    Pin sda, scl; // pin definitions must be kept in this order
    uint16_t rate;

    void init (char const* desc, uint16_t khz =400);

    void deinit () {
        Pin::config(":F,:U", &sda, 2); // keep SCL pulled up
    }

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

struct Base {
    using ID = uint8_t;

    virtual bool start (uint8_t addr) =0;
    virtual void stop () =0;
    virtual int rdByte (bool last) =0;
    virtual bool wrByte (uint8_t data) =0;
    virtual bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const =0;
};

template< typename I2C >
struct Wrap final : Base, I2C {
    bool start (uint8_t a) override { return I2C::start(a); }
    void stop () override { I2C::stop(); }
    int rdByte (bool l) override { return I2C::rdByte(l); }
    bool wrByte (uint8_t d) override { return I2C::wrByte(d); }
    bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const override {
        return I2C::transfer(a, m, p, n);
    }
};

} // namespace jeeh
