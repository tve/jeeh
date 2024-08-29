// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "convol"

//CG3 board leds
#define LED  "A5"
#define LED1 "A5"
#define LED2 "B13"

constexpr Pin led (LED1), ledN (LED2);

// TODO it looks like the entire "G" port on my disco board is broken ...
//constexpr Pin dcfData ("G12"), dcfPon ("G10");
constexpr Pin dcfData ("B2"), dcfPon ("H2");

namespace serio {
    enum { CR1=0x00, BRR=0x0C, ISR=0x1C, RDR=0x24, TDR=0x28 };

    void init () {
        Pin::config("A2:7");
        RCC(ena::USART2,1) = 1;
        USART2[BRR] = SystemCoreClock / 1'000'000;
        USART2[CR1] = (1<<3) | (1<<2) | (1<<0); // TE RE UE
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
    RCC[0x00](2) = 1; // MSIPLLEN
    fastClock(false); // 48 MHz MSI
    assert(RCC[0x00](2));

    cycles::init();
    cycles::msBusy(10); // to let MSI PLL stabilise

    dcfData.mode("U");
    serio::init();

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss);

    dcfPon.mode("P"); // 0 = on
}

void jeeh::logWriter (void const* ptr, size_t len) {
    serio::write(ptr, len);
}
