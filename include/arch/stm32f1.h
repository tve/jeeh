static void enableClkWithPll (int freq) {
    (void) freq; // TODO ignored for now
    FLASH[0x00] = 0x12;           // flash acr, two wait states
    RCC[0x00](16) = 1;            // rcc cr, set HSEON
    while (RCC[0x00](17) == 0) {} // wait for HSERDY
    // 8 MHz xtal src, pll 9x, pclk1 = hclk/2, adcpre = pclk2/6 [1] pp.100
    RCC[0x04] = (7<<18) | (1<<16) | (2<<14) | (4<<8) | (2<<0);
    RCC[0x00](24) = 1;            // rcc cr, set PLLON
    while (RCC[0x00](25) == 0) {} // wait for PLLRDY
}

uint32_t fastClock (bool pll) {
    (void) pll; // TODO always true for now
    enableClkWithPll(F_CPU/1'000'000);
    return SystemCoreClock = F_CPU;
}

namespace rtc {
    enum { BDCR=0x20, CRL=0x04, PRLL=0x0C, CNTH=0x18, CNTL=0x1C };

void init (bool lse) {
    assert(lse); // TODO
    RCC(ena::PWR, 1) = 1;
    PWR[0x00](8) = 1; // DBP
    RCC(ena::BKP, 1) = 1;

    RCC[BDCR](0) = 1;               // LSEON
    while (RCC[BDCR](1) == 0) {}    // wait for LSRDY
    RCC[BDCR](8) = 1;               // RTSEL = LSE
    RCC[BDCR](15) = 1;              // RTCEN
    RTC[CRL](3) = 0;                // ~RSF
    while (RTC[CRL](3) == 0) {}     // wait for RSF
    RTC[CRL](4) = 1;                // CNF
    RTC[PRLL] = 32767;              // 32 kHz crystal
    RTC[CRL](4) = 0;                // ~CNF
    while (RTC[CRL](5) == 0) {}     // wait for RTOFF

}

DateTime getDate () {
    return DateTime { getSecs() };
}

uint32_t getSecs () {
    while (true) {
        uint16_t lo = RTC[CNTL];
        uint16_t hi = RTC[CNTH];
        if (lo == RTC[CNTL])
            return lo | (hi<<16);
        // if low word changed, try again
    }
}

void set (DateTime const& dt) {
    set((uint32_t) dt);
}

void set (uint32_t t) {
    RTC[CRL](4) = 1;            // CNF
    RTC[CNTL] = (uint16_t) t;
    RTC[CNTH] = t >> 16;
    RTC[CRL](4) = 0;            // ~CNF
    while (RTC[CRL](5) == 0) {} // wait for RTOFF
}

uint32_t getReg (int reg) {
    return BKP[4*reg];
}

void setReg (int reg, uint32_t val) {
    BKP[4*reg] = val;
}

} // namespace rtc
