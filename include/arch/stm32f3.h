static void enableClkWithPll (int freq) {
    auto div = freq/8 - 2;
    FLASH[0x00] = 0x12;            // ACR two wait states
#if XTAL == 0
    RCC[0x04] = (div<<18) | (1<<15) | (4<<8); // HSI
#else
    (void) freq; // TODO ignored for now
    RCC[0x00](16) = 1;             // CR HSEON
    while (RCC[0x00](17) == 0) {}  // CR wait for HSERDY
    // 8 MHz xtal src, pll 9x, pclk1 = hclk/2, adcpre = pclk2/6 [1] pp.100
    RCC[0x04] = (div<<18) | (1<<16) | (4<<8) | (1<<0);
#endif
    RCC[0x00](24) = 1;             // CR PLLON
    while (RCC[0x00](25) == 0) {}  // CR wait for PLLRDY
    RCC[0x04](0,2) = 2;            // CFGR SW=PLL
    //while (RCC[0x04](2,2) != 2) {} // CFGR wait for SWS
    //RCC[0x00](0) = 0;              // ~HSION
}

uint32_t fastClock (bool pll) {
    (void) pll; // TODO always true for now
    enableClkWithPll(F_CPU/1'000'000);
    return SystemCoreClock = F_CPU;
}
