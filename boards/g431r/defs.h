// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "serial"

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

//CG[ board uarta
#define UARTA_NAME USART1
#define UARTA_TRIGGER(w) extern "C" { \
    void DMA1_Channel3_IRQHandler () { (w).irqTx(); } \
    void DMA1_Channel4_IRQHandler () { (w).irqRx(); } \
    void USART1_IRQHandler () { (w).irqRx(); } \
}
constexpr uart::Config UARTA_CONF {
    "C4:U7,C5", USART1.ADDR, ena::USART1, 170,
    DMA1.ADDR, 1-1, 3-1,4-1, 25,24,
    Irq::DMA1_CH3, Irq::DMA1_CH4, Irq::USART1, 64,
};
//CG]

//CG[ board uartb
#define UARTB_NAME USART3
#define UARTB_TRIGGER(w) extern "C" { \
    void DMA1_Channel5_IRQHandler () { (w).irqTx(); } \
    void DMA1_Channel6_IRQHandler () { (w).irqRx(); } \
    void USART3_IRQHandler () { (w).irqRx(); } \
}
constexpr uart::Config UARTB_CONF {
    "B9:U7,B8", USART3.ADDR, ena::USART3, 170,
    DMA1.ADDR, 1-1, 5-1,6-1, 29,28,
    Irq::DMA1_CH5, Irq::DMA1_CH6, Irq::USART3, 64,
};
//CG]

//CG[ board uartc
#define UARTC_NAME UART4
#define UARTC_TRIGGER(w) extern "C" { \
    void DMA2_Channel1_IRQHandler () { (w).irqTx(); } \
    void DMA2_Channel2_IRQHandler () { (w).irqRx(); } \
    void UART4_IRQHandler () { (w).irqRx(); } \
}
constexpr uart::Config UARTC_CONF {
    "C10:U5,C11", UART4.ADDR, ena::UART4, 170,
    DMA2.ADDR, 2-1, 1-1,2-1, 31,30,
    Irq::DMA2_CH1, Irq::DMA2_CH2, Irq::UART4, 64,
};
//CG]

//CG[ board uartd
#define UARTD_NAME LPUART1
#define UARTD_TRIGGER(w) extern "C" { \
    void DMA2_Channel3_IRQHandler () { (w).irqTx(); } \
    void DMA2_Channel4_IRQHandler () { (w).irqRx(); } \
    void LPUART1_IRQHandler () { (w).irqRx(); } \
}
constexpr uart::Config UARTD_CONF {
    "C1:U8,C0", LPUART1.ADDR, ena::LPUART1, 170,
    DMA2.ADDR, 2-1, 3-1,4-1, 35,34,
    Irq::DMA2_CH3, Irq::DMA2_CH4, Irq::LPUART1, 64,
};
//CG]

//Dev<uart::Poll<UART_CONF>> console;
Dev<uart::Sync<UART_CONF>> console;
//Dev<uart::Async<UART_CONF>> console;
//UART_TRIGGER(console)

Dev<uart::Poll<UARTA_CONF>> uartA;
//UARTA_TRIGGER(uartA)

Dev<uart::Poll<UARTB_CONF>> uartB;
//UARTB_TRIGGER(uartB)

Dev<uart::Poll<UARTC_CONF>> uartC;
//UARTC_TRIGGER(uartC)

Dev<uart::Poll<UARTD_CONF>> uartD;
//UARTD_TRIGGER(uartD)

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1 || fd == 2)
        console.write(buf, len);
    return len;
}

void initBoard () {
    //fastClock(); // 160 MHz
    cycles::init();

    console.init(1'000'000);
    uartA.init(115'200);
    uartB.init(115'200);
    uartC.init(115'200);
    uartD.init(115'200/256); // LPUART

    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
    logf("\t uart c=%s  a=%s  b=%s  c=%s  d=%s",
            UART_CONF.pins, UARTA_CONF.pins, UARTB_CONF.pins,
            UARTC_CONF.pins, UARTD_CONF.pins);
}
