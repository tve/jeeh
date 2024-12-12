// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "lcd"

//CG1 board leds
#define LED  "A5"

Pin led (LED,"P");

//CG[ board uart2
#define UART2_NAME USART2
#define UART2_TRIGGER(w) extern "C" { \
    void USART2_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Channel1_IRQHandler () { (w).irqDma(); } \
    void DMA1_Channel2_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART2_CONF {
    "A2:7,A3", USART2.ADDR, ena::USART2, 170,
    DMA1.ADDR, 1-1, 1-1,2-1, 27,26,
    Irq::DMA1_CH1, Irq::DMA1_CH2, Irq::USART2,
};
//CG]

Dev<uart::Async<UART2_CONF>> console;
UART2_TRIGGER(console)

//CG[ board spi
#define SPI_NAME SPI1
#define SPI_TRIGGER(w) extern "C" { \
    void DMA1_Channel3_IRQHandler () { (w).irqDma(); } \
    void DMA1_Channel4_IRQHandler () { (w).irqDma(); } \
}
constexpr spi::Config SPI_CONF {
    "A7:H5,A6,A5,B6:HP", SPI1.ADDR, ena::SPI1, 170,
    DMA1.ADDR, 1-1, 3-1,4-1, 11,10,
    Irq::DMA1_CH3, Irq::DMA1_CH4,
};
//CG]

void initBoard () {
    fastClock(); // 160 MHz
    cycles::init();
    rtc::init();

    console.init(10'000'000);

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, dt.ms);
}

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        console.write(buf, len);
    return len;
}
