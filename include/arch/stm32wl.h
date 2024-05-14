static void enableClockHSI () { // using 16 MHz HSI
    RCC[0x00](8) = 1;             // HSION
    while (RCC[0x00](10) == 0) {} // wait for HSIRDY
    RCC[0x08](0, 2) = 1;          // HSI as SYSCLK
    PWR[0x00](9, 2) = 2;          // VOS range 2
    while (PWR[0x14](10) != 0) {} // wait for ~VOSF
    FLASH[0x00](0, 3) = 0;        // flash ACR, 0 wait states
}

static void enableClkMaxMSI () { // using internal 48 MHz MSI
    FLASH[0x00](0, 3) = 2;           // flash ACR, 2 wait states
    while (FLASH[0x00](0,3) != 2) {} // wait until wait matches
    PWR[0x00](9, 2) = 1;             // VOS range 1
    while (PWR[0x14](10) != 0) {}    // wait for ~VOSF
    RCC[0x00](0) = 1;                // MSION
    while (RCC[0x00](1) == 0) {}     // wait for MSIRDY
    RCC[0x00](3, 5) = 0b10111;       // MSI 48 MHz
    RCC[0x08](0, 2) = 0;             // MSI as SYSCLK
}

static void enableClkSaver (int range) { // using MSI at 100 kHz to 16 MHz
    // note: above 16 Mhz requires flash wait states
    RCC[0x00](0) = 1;                // MSION
    while (RCC[0x00](1) == 0) {}     // wait for MSIRDY
    RCC[0x00] = (range<<4)|(1<<3)|1; // MSI 48 MHz, ~HSION, ~PLLON, MSION
    RCC[0x08] = 0b00;                // MSI as SYSCLK
    FLASH[0x00] = 0;                 // no ACR, no wait states (up to 6Mhz)
}

uint32_t fastClock (bool high) {
    PWR[0x00](9, 2) = 0b01;        // VOS range 1
    while (PWR[0x14](10) != 0) {}  // wait for ~VOSF
    if (high) enableClkMaxMSI(); else enableClockHSI();
    return SystemCoreClock = high ? 48'000'000 : 16'000'000;
}

uint32_t slowClock (bool high) {
    enableClkSaver(high ? 0b0110 : 0b0000); // MSI 4 MHz / 100 kHz
    PWR[0x00](9, 2) = 0b10;         // VOS range 2
    return SystemCoreClock = high ? 4'000'000 : 100'000;
}
