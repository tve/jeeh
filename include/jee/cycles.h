// Hardware cycle counter (not present on M0+).

namespace jeeh::cycles {

constexpr IoReg<0xE000'1000> DWT {};

enum { CTRL=0x000, CYCCNT=0x004, LAR=0xFB0, DEMCR=0x0FC };

inline static void clear () {
    DWT[0x04] = 0;
}

inline static void init () {
    SCB[DEMCR](24) = 1; // TRCENA
    DWT[LAR] = 0xC5ACCE55;
    clear();
    DWT[CTRL](0) = 1;
}

inline static void deinit () {
    DWT[CTRL](0) = 0;
}

inline static uint32_t count () {
    return DWT[0x04];
}

inline static uint32_t millis () {
    return count() / (SystemCoreClock/1000);
}

inline static uint32_t micros () {
    // scaled to work with any clock rate multiple of 100 kHz
    return (10*count()) / (SystemCoreClock/100'000);
}

inline static void msBusy (uint32_t ms) {
    auto n = ms * (SystemCoreClock/1000);
    auto t = count();
    while (count() - t < n) {}
}

} // namespace jeeh::cycles
