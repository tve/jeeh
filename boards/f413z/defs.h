// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "uarts"

//CG[ board leds
#define LED  "B0"
#define LED1 "B0"
#define LED2 "B7"
#define LED3 "B14"
//CG]

const Pin led (LED,"P");

//CG[ board uart
#define UART_NAME USART3
#define UART_TRIGGER(w) extern "C" { \
    void USART3_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Stream3_IRQHandler () { (w).irqDma(); } \
    void DMA1_Stream1_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART_CONF {
    "D8:U7,D9", USART3.ADDR, ena::USART3, 50,
    DMA1.ADDR, 1-1, 3-0,1-0, 4,4,
    Irq::DMA1_Stream3, Irq::DMA1_Stream1, Irq::USART3,
};
//CG]
//CG[ board uart1
#define UART1_NAME USART1
#define UART1_TRIGGER(w) extern "C" { \
    void USART1_IRQHandler () { (w).irqIdle(); } \
    void DMA2_Stream7_IRQHandler () { (w).irqDma(); } \
    void DMA2_Stream2_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART1_CONF {
    "B6:U7,B3", USART1.ADDR, ena::USART1, 100,
    DMA2.ADDR, 2-1, 7-0,2-0, 4,4,
    Irq::DMA2_Stream7, Irq::DMA2_Stream2, Irq::USART1,
};
//CG]
//CG[ board uart2
#define UART2_NAME USART2
#define UART2_TRIGGER(w) extern "C" { \
    void USART2_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Stream6_IRQHandler () { (w).irqDma(); } \
    void DMA1_Stream5_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART2_CONF {
    "A2:U7,A3", USART2.ADDR, ena::USART2, 50,
    DMA1.ADDR, 1-1, 6-0,5-0, 4,4,
    Irq::DMA1_Stream6, Irq::DMA1_Stream5, Irq::USART2,
};
//CG]
//CG[ board uart4
#define UART4_NAME UART4
#define UART4_TRIGGER(w) extern "C" { \
    void UART4_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Stream4_IRQHandler () { (w).irqDma(); } \
    void DMA1_Stream2_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART4_CONF {
    "A0:U8,C11", UART4.ADDR, ena::UART4, 50,
    DMA1.ADDR, 1-1, 4-0,2-0, 4,4,
    Irq::DMA1_Stream4, Irq::DMA1_Stream2, Irq::UART4,
};
//CG]
//CG[ board uart5
#define UART5_NAME UART5
#define UART5_TRIGGER(w) extern "C" { \
    void UART5_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Stream7_IRQHandler () { (w).irqDma(); } \
    void DMA1_Stream0_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART5_CONF {
    "C12:U8,D2", UART5.ADDR, ena::UART5, 50,
    DMA1.ADDR, 1-1, 7-0,0-0, 8,4,
    Irq::DMA1_Stream7, Irq::DMA1_Stream0, Irq::UART5,
};
//CG]
//CG[ board uart6
#define UART6_NAME USART6
#define UART6_TRIGGER(w) extern "C" { \
    void USART6_IRQHandler () { (w).irqIdle(); } \
    void DMA2_Stream6_IRQHandler () { (w).irqDma(); } \
    void DMA2_Stream1_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART6_CONF {
    "G14:U8,G9", USART6.ADDR, ena::USART6, 100,
    DMA2.ADDR, 2-1, 6-0,1-0, 5,5,
    Irq::DMA2_Stream6, Irq::DMA2_Stream1, Irq::USART6,
};
//CG]
//CG[ board uart9
#define UART9_NAME UART9
#define UART9_TRIGGER(w) extern "C" { \
    void UART9_IRQHandler () { (w).irqIdle(); } \
    void DMA2_Stream0_IRQHandler () { (w).irqDma(); } \
    void DMA2_Stream7_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART9_CONF {
    "D15:U11,D14", UART9.ADDR, ena::UART9, 100,
    DMA2.ADDR, 2-1, 0-0,7-0, 1,0,
    Irq::DMA2_Stream0, Irq::DMA2_Stream7, Irq::UART9,
};
//CG]
//CG[ board uart10
#define UART10_NAME UART10
#define UART10_TRIGGER(w) extern "C" { \
    void UART10_IRQHandler () { (w).irqIdle(); } \
    void DMA2_Stream5_IRQHandler () { (w).irqDma(); } \
    void DMA2_Stream3_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART10_CONF {
    "E3:U11,E2", UART10.ADDR, ena::UART10, 100,
    DMA2.ADDR, 2-1, 5-0,3-0, 9,9,
    Irq::DMA2_Stream5, Irq::DMA2_Stream3, Irq::UART10,
};
//CG]

namespace serio {
    enum { ISR=0x00, TDR=0x04, BRR=0x08, CR1=0x0C };

    void init () {
        Pin::config("D8:7");
        RCC(ena::UART_NAME,1) = 1;
        UART_NAME[BRR] = (SystemCoreClock/2) / 1'000'000; // 50 MHz APB bus
        UART_NAME[CR1] = (1<<13) | (1<<3); // UE TE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!UART_NAME[ISR](7)) {} // TXE
            UART_NAME[TDR] = ((uint8_t const*) ptr)[i];
        }
        //while (!UART_NAME[serio::ISR](6)) {} // TC
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
    fastClock(); // 100 MHz
    cycles::init();
    rtc::init(false); // TODO no on-board xtal?

    serio::init();

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, dt.ms);
}
