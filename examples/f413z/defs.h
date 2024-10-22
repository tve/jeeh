// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "blink"

//CG[ board leds
#define LED  "B0"
#define LED1 "B0"
#define LED2 "B7"
#define LED3 "B14"
//CG]

const Pin led (LED,"P");

//CG[ board uart
#define UART_NAME  USART3
#define UART_PINS  "D8:7,D9"
#define UART_FREQ  50
#define UART_CONF  Irq::DMA1_Stream3,Irq::DMA1_Stream1,1-1,3-0,1-0,4,4
//CG]

namespace serio {
    enum { ISR=0x00, TDR=0x04, BRR=0x08, CR1=0x0C };

    void init () {
        Pin::config("D8:7");
        RCC(ena::USART3,1) = 1;
        // SystemCoreClock mixup? gcc inits as 16 MHz, but Nucleo-144 is 8 MHz
        //  (normally, this next divider should be 100/2)
        // TODO can probably also be fixed by using HSI16 iso HSE8
        USART3[BRR] = SystemCoreClock / 4 / 1'000'000; // 100/4 MHz APB bus
        USART3[CR1] = (1<<13) | (1<<3); // UE TE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!USART3[ISR](7)) {} // TXE
            USART3[TDR] = ((uint8_t const*) ptr)[i];
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
    fastClock(); // 100 MHz
    cycles::init();
    rtc::init(false); // TODO no on-board xtal?

    serio::init();

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (dt.ff * 1000) / 256);
}
