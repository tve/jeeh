// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "app"

//CG3 board leds
#define LED  "A5"
#define LED1 "A5"
#define LED2 "A1"

const Pin led1 (LED1,"P"); // green
const Pin led2 (LED2,"P"); // orange

//CG[ board uart1
#define UART1_NAME USART1
#define UART1_TRIGGER(w) extern "C" { \
    void USART1_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Channel4_IRQHandler () { (w).irqDma(); } \
    void DMA1_Channel5_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART1_CONF {
    "A9:7,A10", USART1.ADDR, ena::USART1, 72,
    DMA1.ADDR, 1-1, 4-1,5-1, 2,2,
    Irq::DMA1_CH4, Irq::DMA1_CH5, Irq::USART1,
};
//CG]

uart::Async<UART1_TYPE> gpsUart (UART1_CONF);
UART1_TRIGGER(gpsUart)

//CG[ board uart2
#define UART2_NAME USART2
#define UART2_TRIGGER(w) extern "C" { \
    void USART2_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Channel7_IRQHandler () { (w).irqDma(); } \
    void DMA1_Channel6_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART2_CONF {
    "A2:7,A3", USART2.ADDR, ena::USART2, 36,
    DMA1.ADDR, 1-1, 7-1,6-1, 2,2,
    Irq::DMA1_CH7, Irq::DMA1_CH6, Irq::USART2,
};
//CG]

uart::Async<UART2_TYPE> ttyUart (UART2_CONF);
UART2_TRIGGER(ttyUart)

const Pin gpsPps ("B5","U2"), // D4 TIM3-CH2
          lseOut ("A8","P0"), // D6 MCO
          lseIn  ("A0","U1"); // D2 TIM2-ETR

const Pin dcfOff ("B7","P"), // D9
          dcfDat ("A4","U"), // D10
          dcfGnd ("A7","P"), // D11
          dcfVcc ("A6","P"); // D12

const Pin msfVcc ("C3","P"), // A3
          msfDat ("C2","U"), // A2
          msfOff ("C1","P"), // A1
          msfGnd ("C0","P"); // A0

void initBoard () {
    fastClock(); // 72 MHz
    cycles::init();
    rtc::init();

    ttyUart.init(1'000'000);
    ttyUart.setName("tty-uart");

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate().asText();
    logf("\n%s: %s @ %d MHz - %s",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000, dt.buf);
}

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        ttyUart.write(buf, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    ttyUart.write(ptr, len);
}
