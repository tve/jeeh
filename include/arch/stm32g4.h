#ifndef XTAL
#define XTAL 8
#endif

static void enableHSI () {
    RCC[0x00](8) = 1;             // HSION
    while (RCC[0x00](10) == 0) {} // wait for HSIRDY
    RCC[0x08](0,2) = 1;           // switch to HSI
}

static void enableClkWithPll (int freq) {
    auto wait = (freq-1) / 30;
    if (freq > 150) { // needs special boost mode for voltage scaling
        wait = (freq-1) / 34;
        PWR[0x80] = 0;                  // R1MODE boost
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
    enableHSI();
    RCC[0x0C] = (2<<27) | (1<<24) | ((freq/2)<<8) | ((16/4-1)<<4) | (2<<0);
#endif
    RCC[0x00](24) = 1;                  // PLLON
    while (RCC[0x00](25) == 0) {}       // wait for PLLRDY
    RCC[0x08](0,2) = 3;                 // switch to PLL
}

uint32_t fastClock (bool high) {
    if (high) {
        enableClkWithPll(F_CPU/1'000'000);
        return clockChange(F_CPU);
    } else {
        enableHSI();
        return clockChange(16'000'000);
    }
}

namespace flash {
    enum { KEYR=0x08, SR=0x10, CR=0x14 };

    uint32_t pageSize (uint32_t offset) {
        (void) offset;
        return 2048;
    }

    volatile uint32_t& word (uint32_t pos) {
        return *(uint32_t*) (0x08000000 + pos);
    }

    void wait () {
        while (FLASH[SR](16)) {}
    }

    void unlock () {
        wait();
        if (FLASH[CR](31)) {
            FLASH[KEYR] = 0x45670123;
            FLASH[KEYR] = 0xCDEF89AB;
        }
    }

    void finish () {
        wait();
        FLASH[CR] = 1<<31; // LOCK
    }

    void erase (uint32_t offset) {
        //assert(offset < (1<<21)); // offset from flash start, not addr
        //assert(offset % pageSize(offset) == 0);
        auto sector = offset / pageSize(offset);
        unlock();
        FLASH[CR] = (sector<<3) | (1<<1); // SNB PER
        FLASH[CR](16) = 1; // STRT
        finish();
    }

    void write2w (uint32_t offset, uint32_t val1, uint32_t val2) {
        //assert(offset < (1<<21));      // offset from flash start, not addr
        //assert(offset % 8 == 0);       // must be on 8-byte boundary
        //assert(word(offset) == ~0U);   // must be empty at offset
        //assert(word(offset+4) == ~0U); // must be empty at offset+4
        unlock();
        FLASH[CR](0) = 1; // PG
        word(offset) = val1;
        word(offset+4) = val2;
        wait();
    }

    void write8w (uint32_t offset, uint32_t const* data) {
        for (auto i = 0U; i < 8; i += 2) {
            write2w(offset, data[i], data[i+1]);
            offset += 8;
        }
    }
}
