// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "tick"

//CG1 board leds
#define LED  "B3"

const Pin led (LED,"P");

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:U4,A3"
#define UART_FREQ  32
#define UART_TYPE  USART2.ADDR, DMA1.ADDR, 4-1, 5-1
#define UART_CONF  { ena::USART2, 32, Irq::USART2, \
                     Irq::DMA1_Channel4, Irq::DMA1_Channel5, { 1-1,4,4 } }
#define UART_IRQS(name) extern "C" { \
    void USART2_IRQHandler () { name.idleIrq(); } \
    void DMA1_Stream4_IRQHandler () { name.dmaIrq(); } \
    void DMA1_Stream5_IRQHandler () { name.dmaIrq(); } \
}
//CG]

namespace serio {
    enum { CR1=0x00, BRR=0x0C, ISR=0x1C, TDR=0x28 };

    void init () {
        Pin::config("A2:U4");
        RCC(ena::USART2,1) = 1;
        USART2[BRR] = SystemCoreClock / 115'200;
        USART2[CR1] = (1<<3) | (1<<0); // TE UE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!USART2[ISR](7)) {} // TXE
            USART2[TDR] = ((uint8_t const*) ptr)[i];
        }
        //while (!USART2[serio::ISR](6)) {} // TC
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
    fastClock(); // 32 MHz
    rtc::init();

    serio::init();

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (dt.ff * 1000) / 256);
}
