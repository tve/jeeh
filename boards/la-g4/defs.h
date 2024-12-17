// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "async"

//CG1 board leds
#define LED  "B8"

const Pin led (LED,"P");

//CG[ board uart
#define UART_NAME USART2
#define UART_TRIGGER(w) extern "C" { \
    void DMA1_Channel1_IRQHandler () { (w).irqTx(); } \
    void DMA1_Channel2_IRQHandler () { (w).irqRx(); } \
    void USART2_IRQHandler () { (w).irqRx(); } \
}
constexpr uart::Config UART_CONF {
    "A2:U7,A3", USART2.ADDR, ena::USART2, 170,
    DMA1.ADDR, 1-1, 1-1,2-1, 27,26,
    Irq::DMA1_CH1, Irq::DMA1_CH2, Irq::USART2, 64,
};
//CG]

Dev<uart::Poll<UART_CONF>> console;
//Dev<uart::Sync<UART_CONF>> console;
//Dev<uart::Async<UART_CONF>> console;
//UART_TRIGGER(console)

//CG[ board uartx
#define UARTX_NAME USART1
#define UARTX_TRIGGER(w) extern "C" { \
    void DMA1_Channel3_IRQHandler () { (w).irqTx(); } \
    void DMA1_Channel4_IRQHandler () { (w).irqRx(); } \
    void USART1_IRQHandler () { (w).irqRx(); } \
}
constexpr uart::Config UARTX_CONF {
    "B6:U7,A10", USART1.ADDR, ena::USART1, 170,
    DMA1.ADDR, 1-1, 3-1,4-1, 25,24,
    Irq::DMA1_CH3, Irq::DMA1_CH4, Irq::USART1, 64,
};
//CG]

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1 || fd == 2)
        console.write(buf, len);
    return len;
}

namespace jeeh {
    Pin tracePins [14];
}

void initBoard () {
    fastClock(); // 160 MHz
    Pin::config("A12:P,B0,B6,A7,A15,B7,A4,A1,A0,A8,A11,B3,B5,B4",
                    tracePins, sizeof tracePins);

    cycles::init();
    console.init(2'000'000);

    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}
