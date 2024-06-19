namespace jeeh {

struct I2cGpio {
    using ID = uint8_t;

    Pin sda, scl; // pin definitions must be kept in this order
    uint16_t rate;

    void init (char const* desc, int r =20);

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

} // namespace jeeh
