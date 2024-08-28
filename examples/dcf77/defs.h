// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "cycles"

//CG3 board leds
#define LED  "A5"
#define LED1 "A5"
#define LED2 "B13"

constexpr Pin led (LED1), ledN (LED2);

// TODO it looks like the entire "G" port on my disco board is broken ...
//constexpr Pin dcfData ("G12");
//constexpr Pin dcfPon ("G10");

constexpr Pin dcfData ("B2"), dcfPon ("H2");

namespace serio {
    enum { CR1=0x00, BRR=0x0C, ISR=0x1C, RDR=0x24, TDR=0x28 };

    void init () {
        Pin::config("A2:7");
        RCC(ena::USART2,1) = 1;
#if 0
        USART2[BRR] = 2 * SystemCoreClock / 2'000'000;
        USART2[CR1] = (1<<15) | (1<<3) | (1<<2) | (1<<0); // OVER8 TE RE UE
#else
        USART2[BRR] = SystemCoreClock / 115'200;
        USART2[CR1] = (1<<3) | (1<<2) | (1<<0); // TE RE UE
#endif
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!USART2[ISR](7)) {} // TXE
            USART2[TDR] = ((uint8_t const*) ptr)[i];
        }
        while (!USART2[serio::ISR](6)) {} // TC
    }
}

void initBoard () {
    led.mode("P");
    ledN.mode("P"); ledN = 1; // inverted logic

    // enable LSE and use it to lock the MSI for better accuracy
    rtc::init();
    //fastClock(); // 80 MHz HSI+PLL
    RCC[0x00](2) = 1; // MSIPLLEN
    fastClock(false); // 48 MHz MSI
    assert(RCC[0x00](2));

    serio::init();
    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);

    cycles::init();

    dcfData.mode("U");
    dcfPon.mode("P"); // 0 = on
}

void jeeh::logWriter (void const* ptr, size_t len) {
    serio::write(ptr, len);
}
