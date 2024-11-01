// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "dcf77"

//CG1 board leds
#define LED  "A1"

Pin led (LED,"P");

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:7,A10"
#define UART_FREQ  72
#define UART_TYPE  USART1.ADDR, DMA1.ADDR, 4-1, 5-1
#define UART_CONF  { ena::USART1, 72, Irq::USART1, \
                     Irq::DMA1_CH4, Irq::DMA1_CH5, { 1-1,2,2 } }
#define UART_INSTALL(name) extern "C" { \
    void USART1_IRQHandler () { name.idleIrq(); } \
    void DMA1_Channel4_IRQHandler () { name.dmaIrq(); } \
    void DMA1_Channel5_IRQHandler () { name.dmaIrq(); } \
}
//CG]

uart::Work<UART_TYPE> gps (UART_CONF);
UART_INSTALL(gps)

Pin gpsPps {"A8","U"}; // D6

Pin dcfPon {"B7","P"}, // D9
    dcfDat {"A4","U"}, // D10
    dcfGnd {"A7","P"}, // D11
    dcfVcc {"A6","P"}; // D12

Pin msfVcc {"C3","P"}, // A3
    msfDat {"C2","U"}, // A2
    msfPon {"C1","P"}, // A1
    msfGnd {"C0","P"}; // A0

namespace serio {
    enum { CR1=0x00, BRR=0x0C, ISR=0x1C, RDR=0x24, TDR=0x28 };

    void init () {
        Pin::config("A2:7");
        RCC(ena::USART2,1) = 1;
        auto hz = SystemCoreClock;
        while (hz > 36'000'000)
            hz /= 2;
        USART2[BRR] = hz / 2'000'000;
        USART2[CR1] = (1<<3) | (1<<0); // TE UE
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
    fastClock(); // 72 MHz MSI
    cycles::init();
    rtc::init();

    serio::init();

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (dt.ff * 1000) / 256);
}

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        serio::write(buf, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    serio::write(ptr, len);
}
