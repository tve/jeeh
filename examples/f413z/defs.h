// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "tick"

//CG[ board leds
#define LED  "B0"
#define LED1 "B0"
#define LED2 "B7"
#define LED3 "B14"
//CG]

const Pin led (LED,"P");

//CG[ board uart
#define UART_NAME  USART3
#define UART_PINS  "D8:U7,D9"
#define UART_FREQ  50
#define UART_TYPE  USART3.ADDR, DMA1.ADDR, 3-0, 1-0
#define UART_CONF  { ena::USART3, 50, Irq::USART3, \
                     Irq::DMA1_Stream3, Irq::DMA1_Stream1, { 1-1,4,4 } }
#define UART_IRQS(name) \
    extern "C" void USART3_IRQHandler () {{ name.idleIrq(); }} \
    extern "C" void DMA1_Stream3_IRQHandler () {{ name.dmaIrq(); }} \
    extern "C" void DMA1_Stream1_IRQHandler () {{ name.dmaIrq(); }}
//CG]
//CG[ board uart1
#define UART1_NAME  USART1
#define UART1_PINS  "B6:U7,B3"
#define UART1_FREQ  100
#define UART1_TYPE  USART1.ADDR, DMA2.ADDR, 7-0, 2-0
#define UART1_CONF  { ena::USART1, 100, Irq::USART1, \
                      Irq::DMA2_Stream7, Irq::DMA2_Stream2, { 2-1,4,4 } }
#define UART1_IRQS(name) \
    extern "C" void USART1_IRQHandler () {{ name.idleIrq(); }} \
    extern "C" void DMA2_Stream7_IRQHandler () {{ name.dmaIrq(); }} \
    extern "C" void DMA2_Stream2_IRQHandler () {{ name.dmaIrq(); }}
//CG]
//CG[ board uart2
#define UART2_NAME  USART2
#define UART2_PINS  "A2:U7,A3"
#define UART2_FREQ  50
#define UART2_TYPE  USART2.ADDR, DMA1.ADDR, 6-0, 5-0
#define UART2_CONF  { ena::USART2, 50, Irq::USART2, \
                      Irq::DMA1_Stream6, Irq::DMA1_Stream5, { 1-1,4,4 } }
#define UART2_IRQS(name) \
    extern "C" void USART2_IRQHandler () {{ name.idleIrq(); }} \
    extern "C" void DMA1_Stream6_IRQHandler () {{ name.dmaIrq(); }} \
    extern "C" void DMA1_Stream5_IRQHandler () {{ name.dmaIrq(); }}
//CG]
//CG[ board uart4
#define UART4_NAME  UART4
#define UART4_PINS  "A0:U8,C11"
#define UART4_FREQ  50
#define UART4_TYPE  UART4.ADDR, DMA1.ADDR, 4-0, 2-0
#define UART4_CONF  { ena::UART4, 50, Irq::UART4, \
                      Irq::DMA1_Stream4, Irq::DMA1_Stream2, { 1-1,4,4 } }
#define UART4_IRQS(name) \
    extern "C" void UART4_IRQHandler () {{ name.idleIrq(); }} \
    extern "C" void DMA1_Stream4_IRQHandler () {{ name.dmaIrq(); }} \
    extern "C" void DMA1_Stream2_IRQHandler () {{ name.dmaIrq(); }}
//CG]
//CG[ board uart5
#define UART5_NAME  UART5
#define UART5_PINS  "C12:U8,D2"
#define UART5_FREQ  50
#define UART5_TYPE  UART5.ADDR, DMA1.ADDR, 7-0, 0-0
#define UART5_CONF  { ena::UART5, 50, Irq::UART5, \
                      Irq::DMA1_Stream7, Irq::DMA1_Stream0, { 1-1,8,4 } }
#define UART5_IRQS(name) \
    extern "C" void UART5_IRQHandler () {{ name.idleIrq(); }} \
    extern "C" void DMA1_Stream7_IRQHandler () {{ name.dmaIrq(); }} \
    extern "C" void DMA1_Stream0_IRQHandler () {{ name.dmaIrq(); }}
//CG]
//CG[ board uart6
#define UART6_NAME  USART6
#define UART6_PINS  "G14:U8,G9"
#define UART6_FREQ  100
#define UART6_TYPE  USART6.ADDR, DMA2.ADDR, 6-0, 1-0
#define UART6_CONF  { ena::USART6, 100, Irq::USART6, \
                      Irq::DMA2_Stream6, Irq::DMA2_Stream1, { 2-1,5,5 } }
#define UART6_IRQS(name) \
    extern "C" void USART6_IRQHandler () {{ name.idleIrq(); }} \
    extern "C" void DMA2_Stream6_IRQHandler () {{ name.dmaIrq(); }} \
    extern "C" void DMA2_Stream1_IRQHandler () {{ name.dmaIrq(); }}
//CG]
//CG[ board uart9
#define UART9_NAME  UART9
#define UART9_PINS  "D15:U11,D14"
#define UART9_FREQ  100
#define UART9_TYPE  UART9.ADDR, DMA2.ADDR, 0-0, 7-0
#define UART9_CONF  { ena::UART9, 100, Irq::UART9, \
                      Irq::DMA2_Stream0, Irq::DMA2_Stream7, { 2-1,1,0 } }
#define UART9_IRQS(name) \
    extern "C" void UART9_IRQHandler () {{ name.idleIrq(); }} \
    extern "C" void DMA2_Stream0_IRQHandler () {{ name.dmaIrq(); }} \
    extern "C" void DMA2_Stream7_IRQHandler () {{ name.dmaIrq(); }}
//CG]
//CG[ board uart10
#define UART10_NAME  UART10
#define UART10_PINS  "E3:U11,E2"
#define UART10_FREQ  100
#define UART10_TYPE  UART10.ADDR, DMA2.ADDR, 5-0, 3-0
#define UART10_CONF  { ena::UART10, 100, Irq::UART10, \
                       Irq::DMA2_Stream5, Irq::DMA2_Stream3, { 2-1,9,9 } }
#define UART10_IRQS(name) \
    extern "C" void UART10_IRQHandler () {{ name.idleIrq(); }} \
    extern "C" void DMA2_Stream5_IRQHandler () {{ name.dmaIrq(); }} \
    extern "C" void DMA2_Stream3_IRQHandler () {{ name.dmaIrq(); }}
//CG]

namespace serio {
    enum { ISR=0x00, TDR=0x04, BRR=0x08, CR1=0x0C };

    void init () {
        Pin::config("D8:7");
        RCC(ena::UART_NAME,1) = 1;
        // SystemCoreClock mixup? gcc inits as 16 MHz, but Nucleo-144 is 8 MHz
        //  (normally, this next divider should be 100/2)
        // TODO can probably also be fixed by using HSI16 iso HSE8
        UART_NAME[BRR] = SystemCoreClock / 4 / 1'000'000; // 100/4 MHz APB bus
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
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (dt.ff * 1000) / 256);
}
