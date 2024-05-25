static void enableHSI () { // using 16 MHz HSI
    RCC[0x00](8) = 1;             // HSION
    while (RCC[0x00](10) == 0) {} // wait for HSIRDY
    RCC[0x08](0, 2) = 1;          // HSI as SYSCLK
    PWR[0x00](9, 2) = 2;          // VOS range 2
    while (PWR[0x14](10) != 0) {} // wait for ~VOSF
    FLASH[0x00](0, 3) = 0;        // flash ACR, 0 wait states
}

static void enableMaxMSI () { // using internal 48 MHz MSI
    FLASH[0x00](0, 3) = 2;           // flash ACR, 2 wait states
    while (FLASH[0x00](0,3) != 2) {} // wait until wait matches
    PWR[0x00](9, 2) = 1;             // VOS range 1
    while (PWR[0x14](10) != 0) {}    // wait for ~VOSF
    RCC[0x00](0) = 1;                // MSION
    while (RCC[0x00](1) == 0) {}     // wait for MSIRDY
    RCC[0x00](3, 5) = 0b10111;       // MSI 48 MHz
    RCC[0x08](0, 2) = 0;             // MSI as SYSCLK
}

static void enableMSI (int range) { // using MSI at 100 kHz to 16 MHz
    // note: above 6 Mhz requires flash wait states using voltage-scale-2
    RCC[0x00](0) = 1;                // MSION
    while (RCC[0x00](1) == 0) {}     // wait for MSIRDY
    RCC[0x00] = (range<<4)|(1<<3)|1; // MSI X MHz, ~HSION, ~PLLON, MSION
    RCC[0x08] = 0b00;                // MSI as SYSCLK
    FLASH[0x00] = range < 7 ? 0 : 1; // wait state > 6Mhz
}

// fastClock(true): 48Mhz (max clock) using HSE 'cause radio requires HSE...
// fastClock(false): 16Mhz HSI 'cause peripherals can be clocked by HSI
uint32_t fastClock (bool high) {
    PWR[0x00](9, 2) = 0b01;        // VOS range 1
    while (PWR[0x14](10) != 0) {}  // wait for ~VOSF
    if (high) enableMaxMSI(); else enableHSI();
    return clockChange(high ? 48'000'000 : 16'000'000);
}

// slowClock(true): 8Mhz using MSI 'cause efficient
// slowClock(false): 100khz using MSI 'cause least power consumption w/out stopping
uint32_t slowClock (bool high) {
    enableMSI(high ? 0b0111 : 0b0000); // MSI 8 MHz / 100 kHz
    PWR[0x00](9, 2) = 0b10;         // VOS range 2
    return clockChange(high ? 8'000'000 : 100'000);
}
