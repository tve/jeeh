// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "sram-poll"

const Pin led {"C13","P"};    // push-pull output mode
const Pin button {"A0","U"};  // pull-up input mode

namespace serio {
    enum { SR=0x00, DR=0x04, BRR=0x08, CR1=0x0C };

    void init () {
        Pin::config("A9:U7");
        RCC(ena::USART1,1) = 1;
        USART1[BRR] = (SystemCoreClock/2) / 1'000'000;
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

void initBoard () {
    fastClock();
    cycles::init();
    serio::init();

    logf("%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}
