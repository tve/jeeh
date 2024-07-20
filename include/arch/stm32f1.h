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

// On F1, the RTC cannot generate subsecond interrupts unless the prescaler is
// set to run faster than 1 Hz. To resemble newer µCs, it is set here to run
// at 250 Hz. This means that the 32-bit RTC counter overflows every ≈199 days.
// It also increases power consumption (but F1 is not meant for low-power use).

enum {
    CRH=0x00,CRL=0x04,PRLL=0x0C,DIVL=0x10,CNTH=0x18,CNTL=0x1C,BDCR=0x20,CSR=0x24
};

uint32_t offSecs; // offset, since the RTC will run at 250 Hz iso 1 Hz

void init (bool lse) {
    RCC(ena::PWR, 1) = 1;
    PWR[0x00](8) = 1; // DBP
    RCC(ena::BKP, 1) = 1;

    if (lse) {
        RCC[BDCR](0) = 1;            // LSEON
        while (RCC[BDCR](1) == 0) {} // wait for LSERDY
        RCC[BDCR](8,2) = 1;          // RTSEL = LSE
    } else {
        RCC[CSR](0) = 1;             // LSION
        while (RCC[CSR](1) == 0) {}  // wait for LSIRDY
        RCC[BDCR](8,2) = 2;          // RTSEL = LSI
    }
    RCC[BDCR](15) = 1;               // RTCEN
    RTC[CRL](3) = 0;                 // ~RSF
    while (RTC[CRL](3) == 0) {}      // wait for RSF
    RTC[CRL](4) = 1;                 // CNF
    RTC[PRLL] = lse ? 131 : 159;     // 32 kHz crystal or 40 kHz LSI
    RTC[CRL](4) = 0;                 // ~CNF
    while (!RTC[CRL](5)) {}          // wait for RTOFF

}

uint32_t get250hz () {
    while (true) {
        uint16_t lo = RTC[CNTL];
        uint16_t hi = RTC[CNTH];
        if (lo == RTC[CNTL])
            return lo | (hi<<16);
        // if low word changed, try again
    }
}

bool sleep250hz (uint32_t ticks, int mode) {
    (void) ticks; (void) mode;
    fail();
}

bool shortSleep (uint16_t ms, int mode) {
    return sleep250hz(ms/4 + 1, mode);
}

bool longSleep (uint32_t sec, int mode) {
    return sleep250hz(250 * sec, mode);
}

DateTime getDate () {
    auto t = get250hz();
    DateTime dt { offSecs + t/250 };
    dt.ff = t % 250;
    return dt;
}

uint32_t getSecs () {
    return offSecs + get250hz() / 250;
}

void set (DateTime const& dt) {
    set((uint32_t) dt);
}

void set (uint32_t t) {
    offSecs = t;
    while (!RTC[CRL](5)) {} // wait for RTOFF
    RTC[CRL](4) = 1;        // CNF
    RTC[CNTL] = 0;
    RTC[CNTH] = 0;
    RTC[CRL](4) = 0;        // ~CNF
    while (!RTC[CRL](5)) {} // wait for RTOFF
}

uint32_t getReg (int reg) {
    return BKP[4*reg];
}

void setReg (int reg, uint32_t val) {
    BKP[4*reg] = val;
}

} // namespace rtc
