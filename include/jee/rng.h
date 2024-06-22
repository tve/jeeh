// Hardware random number generator.

namespace jeeh::rng {
    enum { CRRCR=0x98 };

    void init () {
        RCC[CRRCR](0) = 1;        // HSI48ON
        while (!RCC[CRRCR](1)) {} // ~HSI48RDY

        RCC(ena::RNG,1) = 1;
        RNG[0x00](2) = 1; // RNGEN
    }

    // return a 32-bit random number (but never zero)
    uint32_t rand () {
        uint32_t r;
        do
            r = RNG[0x08];
        while (r == 0);
        return r;
    }

} // namespace jeeh::rng
