namespace jeeh {

struct I2cGpio {
    Pin sda, scl; // pin definitions must be kept in this order
    uint16_t rate;

    void init (char const* desc, int r =20);

    void deinit () {
        Pin::config(":F,:U", &sda, 2); // keep SCL pulled up
    }

    void detect () const;

    bool start (uint8_t addr) const;
    void stop () const;

    int read (bool last) const;
    bool write (uint8_t data) const;

    int readReg (int addr, int reg) const;
    bool writeReg (int addr, int reg, int val) const;

    bool readRegs (int addr, int reg, void* ptr, int len) const;
    bool writeRegs (int addr, int reg, void const* ptr, int len) const;

private:
    void hold () const {
        for (volatile int i = rate; --i >= 0; ) {}
    }
    void sclLo () const {
        hold(); scl = 0;
    }
    void sclHi () const {
        hold(); scl = 1; for (auto i = 10'000; scl == 0 && i >= 0; --i) {}
    }
};

} // namespace jeeh
