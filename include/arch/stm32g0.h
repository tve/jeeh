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

uint32_t fastClock (bool high) {
    auto wait = high ? 2 : 0;
    FLASH[0x00](0, 3) = wait;         // ACR: 2/0 wait
    while (FLASH[0x00](0, 3) != wait) {}
    //RCC[0x00](8) = 1;                 // CR: HSION
    //while (RCC[0x00](10) == 0) {}     // wait for HSIRDY
    //RCC[0x08](0, 2) = 0;              // SW: HSI
    if (high) {
        RCC[0x0C](0, 2) = 2;          // PLLSRC: HSI
        RCC[0x0C](8, 7) = 8;          // PLLMUL: 16 MHz x 8
        RCC[0x0C](28) = 1;            // PLLREN
        RCC[0x0C](29, 3) = 1;         // PLLDIV: 128 MHz / 2
        RCC[0x00](24) = 1;            // PLLON
        while (RCC[0x00](25) == 0) {} // wait for PLLRDY
        RCC[0x08](0, 2) = 2;          // SW: PLL
    }
    return SystemCoreClock = high ? 64000000 : 16000000;
}
