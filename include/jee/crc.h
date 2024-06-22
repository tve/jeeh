// Access to the built-in 32-bit CRC engine.

namespace jeeh::crc {
    enum { DR=0x00, CR=0x08, INIT=0x10, POL=0x14 };

    void init () {
        RCC(ena::CRC, 1) = 1;
        //CRC[INIT] = ini;
        //CRC[POL] = pol;
        CRC[CR](0) = 1; // RESET
    }

    void deinit () {
        RCC(ena::CRC, 1) = 0;
    }

    void update8 (uint8_t d) { CRC.byte(DR) = d; }
    void update16 (uint16_t d) { CRC.half(DR) = d; }
    void update32 (uint32_t d) { CRC[DR] = d; }

    uint32_t get () { return CRC[DR]; }

} // namespace jeeh::crc
