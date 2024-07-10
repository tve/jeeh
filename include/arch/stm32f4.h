#ifndef XTAL
#define XTAL 8
#endif

static void enableClkWithPll (int freq) {
    FLASH[0x00] = 0x700 + (freq-1)/30;      // flash acr, set wait states
    RCC[0x00](16) = 1;                      // HSEON
    while (RCC[0x00](17) == 0) {}           // wait for HSERDY
    RCC[0x08] = (4<<13) | (5<<10) | (1<<0); // prescaler w/ HSE
    RCC[0x04] = (7<<24) | (1<<22) | ((2*freq)<<6) | (XTAL<<0);
    RCC[0x00](24) = 1;                      // PLLON
    while (RCC[0x00](25) == 0) {}           // wait for PLLRDY
    RCC[0x08] = (4<<13) | (5<<10) | (2<<0);
}

uint32_t fastClock (bool pll) {
    (void) pll; // TODO always true for now
    enableClkWithPll(F_CPU/1'000'000);
    return SystemCoreClock = F_CPU;
}

namespace flash {
    enum { KEYR=0x04, SR=0x0C, CR=0x10 };

    uint32_t pageSize (uint32_t offset) {
        auto kb = offset >> 10;
        return (kb < 64 ? 16 : kb < 128 ? 64 : 128) << 10;
    }

    uint32_t& word (uint32_t pos) {
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
        auto kb = offset >> 10;
        auto sector = kb < 64 ? kb >> 4 : kb < 128 ? 4 : 4 + (kb >> 7);
        unlock();
        FLASH[CR] = // STRT PSIZE SNB SER
                (1<<16) | (2<<8) | (sector<<3) | (1<<1);
        finish();
    }

    void write1w (uint32_t offset, uint32_t val) {
        //assert(offset < (1<<21));      // offset from flash start, not addr
        //assert(offset % 4 == 0);       // must be on 4-byte boundary
        //assert(word(offset) == ~0U);   // must be empty at offset
        unlock();
        FLASH[CR] = (2<<8) | (1<<0); // PSIZE PG
        word(offset) = val;
        wait();
    }

    void write8w (uint32_t offset, uint32_t const* data) {
        for (auto i = 0U; i < 8; ++i) {
            write1w(offset, data[i]);
            offset += 4;
        }
    }
}
