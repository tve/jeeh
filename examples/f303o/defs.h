// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "app"

//CG1 board leds
#define LED  "A1"

Pin led (LED,"P");

//CG[ board uart1
#define UART1_NAME  USART1
#define UART1_PINS  "A9:7,A10"
#define UART1_FREQ  72
#define UART1_TYPE  USART1.ADDR, DMA1.ADDR, 4-1, 5-1
#define UART1_CONF  { ena::USART1, 72, Irq::USART1, \
                      Irq::DMA1_CH4, Irq::DMA1_CH5, { 1-1,2,2 } }
#define UART1_INSTALL(w) extern "C" { \
    void USART1_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Channel4_IRQHandler () { (w).irqDma(); } \
    void DMA1_Channel5_IRQHandler () { (w).irqDma(); } \
}
//CG]

uart::Work<UART1_TYPE> gpsUart (UART1_CONF);
UART1_INSTALL(gpsUart)

//CG[ board uart2
#define UART2_NAME  USART2
#define UART2_PINS  "A2:7,A3"
#define UART2_FREQ  36
#define UART2_TYPE  USART2.ADDR, DMA1.ADDR, 7-1, 6-1
#define UART2_CONF  { ena::USART2, 36, Irq::USART2, \
                      Irq::DMA1_CH7, Irq::DMA1_CH6, { 1-1,2,2 } }
#define UART2_INSTALL(w) extern "C" { \
    void USART2_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Channel7_IRQHandler () { (w).irqDma(); } \
    void DMA1_Channel6_IRQHandler () { (w).irqDma(); } \
}
//CG]

uart::Work<UART2_TYPE> console (UART2_CONF);
UART2_INSTALL(console)

Pin gpsPps {"A8","U"}; // D6

Pin dcfPon {"B7","P"}, // D9
    dcfDat {"A4","U"}, // D10
    dcfGnd {"A7","P"}, // D11
    dcfVcc {"A6","P"}; // D12

Pin msfVcc {"C3","P"}, // A3
    msfDat {"C2","U"}, // A2
    msfPon {"C1","P"}, // A1
    msfGnd {"C0","P"}; // A0

void initBoard () {
    fastClock(); // 72 MHz
    cycles::init();
    rtc::init();

    console.init(UART2_PINS, 1'000'000);

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (dt.ff * 1000) / 256);
}

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        console.write(buf, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    console.write(ptr, len);
}
