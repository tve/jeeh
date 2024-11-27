// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "fram-poll"

const Pin led {"C13","P"};    // push-pull output mode
const Pin button {"A0","U"};  // pull-up input mode

namespace serio {
    enum { SR=0x00, DR=0x04, BRR=0x08, CR1=0x0C };

    void init () {
        Pin::config("A9:U7");
        RCC(ena::USART1,1) = 1;
        USART1[BRR] = SystemCoreClock / 2'000'000;
        USART1[CR1] = (1<<13) | (1<<3); // UE TE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!USART1[SR](7)) {} // TXE
            USART1[DR] = ((uint8_t const*) ptr)[i];
        }
    }
}

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        serio::write(buf, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    serio::write(ptr, len);
}

uint32_t i2cTiming (uint16_t khz, uint16_t mhz =SystemCoreClock/1'000'000) {
    switch (mhz) {
      case 16: // MHz
        switch (khz) {
          //CG[ i2c timing 16
          // 16 Mhz: (remove this line to re-generate)
          case  100: return 0x00504F49; // prs 0 tcd 5 tdd 0 scll 73 sclh 79
          case  400: return 0x00500D12; // prs 0 tcd 5 tdd 0 scll 18 sclh 13
          case 1000: return 0x00500205; // prs 0 tcd 5 tdd 0 scll 5 sclh 2
          //CG]
        }
        break;
      case 100: // MHz
        switch (khz) {
          //CG[ i2c timing 100
          // 100 Mhz: (remove this line to re-generate)
          case  100: return 0x20B0A99A; // prs 2 tcd 11 tdd 0 scll 154 sclh 169
          case  400: return 0x20B02029; // prs 2 tcd 11 tdd 0 scll 41 sclh 32
          case 1000: return 0x00A0222B; // prs 0 tcd 10 tdd 0 scll 43 sclh 34
          //CG]
        }
        break;
    }
    fail();
}

void initBoard () {
    fastClock();
    cycles::init();
    serio::init();

    logf("%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}
