// see https://gcc.gnu.org/onlinedocs/gcc/_005f_005fatomic-Builtins.html

extern "C" uint32_t __atomic_fetch_or_4 (void volatile* p, uint32_t v, int) {
    BlockIRQ irq;
    auto t = *(uint32_t volatile*) p;
    *(uint32_t volatile*) p |= v;
    return t;
}

extern "C" uint32_t __atomic_exchange_4 (void volatile* p, uint32_t v, int) {
    BlockIRQ irq;
    auto t = *(uint32_t volatile*) p;
    *(uint32_t volatile*) p = v;
    return t;
}

uint32_t fastClock (bool pll) {
    FLASH[0x00] = pll ? 0x03 : 0x02;  // ACR: 1/0 wait, enable prefetch
    RCC[0x00](0) = 1;                 // HSION
    while (RCC[0x00](2) == 0) {}      // wait for HSIRDY
    RCC[0x0C](0, 2) = 1;              // switch to HSI
    if (pll) {
        RCC[0x0C](18, 4) = 1;         // PLLMUL
        RCC[0x0C](22, 2) = 1;         // PLLDIV
        RCC[0x00](24) = 1;            // PLLON
        while (RCC[0x00](25) == 0) {} // wait for PLLRDY
        RCC[0x0C](0, 2) = 3;          // switch to PLL
    }
    return SystemCoreClock = pll ? 32000000 : 16000000;
}
