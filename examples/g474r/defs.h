// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "shell"

//CG1 board leds
#define LED  "A5"

Pin led (LED,"P");

//CG[ board uart2
#define UART2_NAME  USART2
#define UART2_PINS  "A2:7,A3"
#define UART2_FREQ  170
#define UART2_TYPE  USART2.ADDR, DMA1.ADDR, 1-1, 2-1
#define UART2_CONF  { ena::USART2, 170, Irq::USART2, \
                      Irq::DMA1_CH1, Irq::DMA1_CH2, { 1-1,27,26 } }
#define UART2_INSTALL(name) extern "C" { \
    void USART2_IRQHandler () { name.idleIrq(); } \
    void DMA1_Channel1_IRQHandler () { name.dmaIrq(); } \
    void DMA1_Channel2_IRQHandler () { name.dmaIrq(); } \
}
//CG]

uart::Work<UART2_TYPE> console (UART2_CONF);
UART2_INSTALL(console)

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "A7:H5,A6,A5,B6:HP"
#define SPI_FREQ  170
#define SPI_TYPE  SPI1.ADDR, DMA1.ADDR, 3-1, 4-1
#define SPI_CONF  { ena::SPI1, 170, \
                    Irq::DMA1_CH3, Irq::DMA1_CH4, { 1-1,11,10 } }
//CG]

void initBoard () {
    fastClock(); // 160 MHz
    cycles::init();
    rtc::init();

    console.init(UART2_PINS, 10'000'000);

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (dt.ff * 1000) / 256);
}

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        console.transfer(true, (uint8_t*) buf, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    console.transfer(true, (uint8_t*) ptr, len);
}
