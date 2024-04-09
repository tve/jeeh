#ifndef XTAL
#define XTAL 8
#endif

static void enableClkWithPll (int freq) {
    auto wait = (freq-1) / 30;
    if (freq > 150) { // needs special boost mode for voltage scaling
        wait = (freq-1) / 34;
        PWR[0x80] = 0;                  // R1MODE boost
        RCC[0x08](4,4) = 8;             // HPRE % 2 (AHB max is 150 MHz)
    }
    FLASH[0x00] = 0x4'0600 | wait;      // flash ACR, set wait states
    while (FLASH[0x00](0,4) != wait) {}
#if XTAL
    RCC[0x00](16) = 1;                  // HSEON
    while (RCC[0x00](17) == 0) {}       // wait for HSERDY
    RCC[0x08](0,2) = 2;                 // no prescaler, switch to HSE
    // XTAL=24: M=6 (VCO_IN=4), N=85 (VCO_OUT=340), R=2 (SYSCLK=170)
    RCC[0x0C] = (2<<27) | (1<<24) | ((freq/2)<<8) | ((XTAL/4-1)<<4) | (3<<0);
#else
    // use HSI16 (xtal is not connected by default on Nucleo-G431KB)
    RCC[0x0C] = (2<<27) | (1<<24) | ((freq/2)<<8) | ((16/4-1)<<4) | (2<<0);
#endif
    RCC[0x00](24) = 1;                  // PLLON
    while (RCC[0x00](25) == 0) {}       // wait for PLLRDY
    RCC[0x08](0,2) = 3;                 // switch to PLL
}

uint32_t fastClock (bool pll) {
    (void) pll; // TODO always true for now
    enableClkWithPll(F_CPU/1'000'000);
    return SystemCoreClock = F_CPU;
}
