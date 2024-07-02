struct Pin {
    uint8_t id =0;

    constexpr Pin () =default;
    explicit constexpr Pin (char const* s) : id (parse(s)) {}

    constexpr int port () const { return id/16-1; }
    constexpr int pin () const { return id%16; }

    [[gnu::always_inline]]
    constexpr auto reg (int off) const { return GPIOA[0x400*port()+off]; }

    bool read () const { return reg(IDR)(pin()); }
    void write (int v) const {
        if constexpr (GPIOA.CAN_BIT_BAND)
            reg(ODR)(pin()) = v;
        else
            reg(BSRR) = ((1<<16) | (v&1)) << pin();
    }
    void set () const { write(1); }
    void clear () const { write(0); }

    // shorthand
    [[gnu::always_inline]]
    void toggle () const {
        if constexpr (GPIOA.CAN_BIT_BAND)
            write(~reg(ODR)(pin()));
        else {
            auto mask = 1 << pin();
            reg(BSRR) = ((mask<<16) | mask) ^ (reg(IDR) & mask);
        }
    }

    operator int () const { return read(); }
    int operator= (int v) const { write(v); return v; }

    // pin definition string: [A-O][<pin#>]:[AFPO][DU][LNHV][<alt#>][,]
    // return -1 on error, 0 if no mode set, or the mode (always > 0)
    int init (char const* desc) {
        if (auto t = parse(desc); t != 0)
            id = t;
        if (id == 0)
            return -1;
        auto i = strcspn(desc, ":,");
        return desc[i] == ':' ? mode(desc+i+1) : 0;
    }

    // reset pin to the default floating input mode
    void deinit () const { mode("F"); }

    // configure multiple pins, return nullptr if ok, else ptr to error
    inline static char const* config (char const* d, Pin* v, int n) {
        Pin dummy;
        int lastMode = 0;
        while (true) {
            if (--n < 0)
                v = &dummy;
            auto m = v->init(d);
            if (m < 0)
                return d;
            if (m != 0)
                lastMode = m;
            else if (lastMode != 0)
                v->mode(lastMode);
            auto p = strchr(d, ',');
            if (p == nullptr)
                return n > 0 ? d : nullptr;
            d = p+1;
            ++v;
        }
    }

    inline static char const* config (char const* d) {
        return config(d, nullptr, 0);
    }

    template< uint32_t N >
    inline static char const* config (char const* d, Pin p [N]) {
        return config(nullptr, p, N);
    }

    int mode (char const* desc) const {
        int m = 0, a = 0;
        for (auto s = desc; *s != ',' && *s != 0; ++s)
            switch (*s) {     // 1 pp ss t mm
                case 'A': m  = 0b1'00'00'0'11; break; // m=11 analog
                case 'F': m  = 0b1'00'00'0'00; break; // m=00 float
                case 'D': m |= 0b1'10'00'0'00; break; // m=00 pull-down
                case 'U': m |= 0b1'01'00'0'00; break; // m=00 pull-up

                case 'P': m  = 0b1'00'01'0'01; break; // m=01 push-pull
                case 'O': m  = 0b1'00'01'1'01; break; // m=01 open drain

                case 'L': m &= 0b1'11'00'1'11; break; // s=00 low speed
                case 'N':                      break; // s=01 normal speed
                case 'H': m ^= 0b0'00'11'0'00; break; // s=10 high speed
                case 'V': m |= 0b0'00'10'0'00; break; // s=11 very high speed

                default:  if (*s < '0' || *s > '9' || a > 1) return -1;
                            m = (m & ~0b11) | 0b10;     // m=10 alt mode
                            a = 10 * a + *s - '0';
                case ',': break; // valid as terminator
            }
        return mode(m + (a<<8));
    }

    int mode (int m) const {
#if STM32F1
        RCC(ena::IOPA+port(), 1) = 1;
        RCC(ena::AFIO, 1) = 1;
        // messy code to keep the mode encoding the same as other families
        auto cr = 0, mm = m&3, t = (m>>2)&1, ss = (m>>3)&3, pp = (m>>5)&3;
        switch (mm) {
            case 0b00: // float or pull-up/-down
                if (pp) {
                    cr = 0b10'00; // input with pull-up/-down
                    reg(ODR)(pin()) = pp & 1;
                } else
                    cr = 0b01'00; // float
                break;
            case 0b01: // output
            case 0b10: // alternate
                cr = ((mm&0b10) | t) << 2;
                cr |= ((0b11'11'01'10) >> (2*ss)) & 3; // low, normal, high, high
                break;
            case 0b11: // analog
                break;
        }
        reg(CRL)(4*pin(), 4) = cr; // CRL/CRH
#else
        enum { TYPER=0x04, OSPEEDR=0x08, PUPDR=0x0C, AFRL=0x20, AFRH=0x24 };

        RCC(ena::GPIOA + port(), 1) = 1;

        auto p = pin();
        reg(AFRL)   (4*p,4) = m >> 8;
        reg(PUPDR)  (2*p,2) = m >> 5;
        reg(OSPEEDR)(2*p,2) = m >> 3;
        reg(TYPER)  (p)     = m >> 2;
        reg(MODER)  (2*p,2) = m;
#endif // STM32F1
        return m;
    }

#if STM32F1
    enum { CRL=0x00, IDR=0x08, ODR=0x0C, BSRR=0x10 };
#else
    enum { MODER=0x00, IDR=0x10, ODR=0x14, BSRR=0x18 };
#endif
private:
    constexpr uint8_t parse (char const* s) {
        if (s == nullptr || *s < 'A' || *s >= 'P')
            return 0;
        uint8_t p = *s++ - '@', pnum = 0;
        while ('0' <= *s && *s <= '9')
            pnum = 10 * pnum + *s++ - '0';
        if (*s != 0 && *s != ':' && *s != ',')
            return 0;
        return 16*p + pnum;
    }
};
