// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "sdspi"

//CG1 board leds
#define LED  "A5"

Pin led (LED,"P");

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:7,A3"
#define UART_FREQ  170
#define UART_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,1-1,2-1,27,26
//CG]

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "A7:H5,A6,A5,B6:HP"
#define SPI_FREQ  170
#define SPI_TYPE  SPI1.ADDR,DMA1.ADDR,3-1,4-1
#define SPI_CONF  {ena::SPI1,170,Irq::DMA1_CH3,Irq::DMA1_CH4}, {1-1,11,10}
//CG]

namespace serio {
    enum { CR1=0x00, BRR=0x0C, ISR=0x1C, RDR=0x24, TDR=0x28 };

    void init () {
        Pin::config(UART_PINS);
        RCC(ena::UART_NAME,1) = 1;
        UART_NAME[BRR] = SystemCoreClock / 10'000'000; // 160 MHz CPU clock
        UART_NAME[CR1] = (1<<29) | (1<<3) | (1<<2) | (1<<0); // FIFOEN TE RE UE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!UART_NAME[ISR](7)) {} // TXE
            UART_NAME[TDR] = ((uint8_t const*) ptr)[i];
        }
        //while (!UART_NAME[serio::ISR](6)) {} // TC
    }
}

void initBoard () {
    fastClock(); // 160 MHz
    serio::init();
    cycles::init();
    rtc::init();

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
    _write(1, (char*) ptr, len);
}
