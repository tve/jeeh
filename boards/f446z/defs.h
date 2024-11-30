// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "lcd"

//CG[ board leds
#define LED  "B0"
#define LED1 "B0"
#define LED2 "B7"
#define LED3 "B14"
//CG]

const Pin led1 (LED1,"P");
const Pin led2 (LED2,"P");
const Pin led3 (LED3,"P");

//CG[ board uart1
#define UART1_NAME  USART6
#define UART1_PINS  "G14:8,G9"
#define UART1_FREQ  180
#define UART1_TYPE  USART6.ADDR, DMA2.ADDR, 7-0, 1-0
#define UART1_CONF  { ena::USART6, 180, Irq::USART6, \
                      Irq::DMA2_Stream7, Irq::DMA2_Stream1, { 2-1,5,5 } }
#define UART1_TRIGGER(w) extern "C" { \
    void USART6_IRQHandler () { (w).irqIdle(); } \
    void DMA2_Stream7_IRQHandler () { (w).irqDma(); } \
    void DMA2_Stream1_IRQHandler () { (w).irqDma(); } \
}
//CG]

uart::Async<UART1_TYPE> gpsUart (UART1_CONF);
UART1_TRIGGER(gpsUart)

//CG[ board uart3
#define UART3_NAME  USART3
#define UART3_PINS  "D8:7,D9"
#define UART3_FREQ  90
#define UART3_TYPE  USART3.ADDR, DMA1.ADDR, 3-0, 1-0
#define UART3_CONF  { ena::USART3, 90, Irq::USART3, \
                      Irq::DMA1_Stream3, Irq::DMA1_Stream1, { 1-1,4,4 } }
#define UART3_TRIGGER(w) extern "C" { \
    void USART3_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Stream3_IRQHandler () { (w).irqDma(); } \
    void DMA1_Stream1_IRQHandler () { (w).irqDma(); } \
}
//CG]

uart::Async<UART3_TYPE> ttyUart (UART3_CONF);
UART3_TRIGGER(ttyUart)

//CG[ board spi1
#define SPI1_NAME  SPI1
#define SPI1_PINS  "A7:5,A6,A5,D14:P"
#define SPI1_FREQ  90
#define SPI1_TYPE  SPI1.ADDR, DMA2.ADDR, 3-0, 2-0
#define SPI1_CONF  { ena::SPI1, 90, \
                     Irq::DMA2_Stream3, Irq::DMA2_Stream2, { 2-1,3,3 } }
//CG]

//spi::Gpio lcdSpi;
spi::Poll<SPI1_NAME.ADDR> lcdSpi (ena::SPI1_NAME, SPI1_FREQ);
//spi::Sync<SPI1_TYPE> lcdSpi (SPI1_CONF);
//spi::Aync<SPI1_TYPE> lcdSpi (SPI1_CONF);
Pin& lcdCmd = lcdSpi.miso; // re-used as C/D output pin
//Pin lcdRst {"D15","P"}; // tied to Vcc instead

#if 0
namespace serio {
    enum { CR1=0x00 , BRR=0x0C, ISR=0x1C, TDR=0x28 };

    void init () {
        Pin::config("D8:U7");
        RCC(ena::USART3,1) = 1;
        USART3[BRR] = SystemCoreClock/2 / 2'000'000;
        USART3[CR1] = (1<<3) | (1<<0); // TE UE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!USART3[ISR](7)) {} // TXE
            USART3[TDR] = ((uint8_t const*) ptr)[i];
        }
        //while (!USART3[serio::ISR](6)) {} // TC
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
#endif

void initBoard () {
    fastClock(); // 180 MHz
    cycles::init();
    rtc::init(false);

    ttyUart.init(UART3_PINS, 2'000'000);
    ttyUart.setName("tty-uart");
    //serio::init();

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, dt.ms);
}

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        ttyUart.write(buf, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    ttyUart.write(ptr, len);
}
