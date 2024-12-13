// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "main"

//CG1 board leds
#define LED  "D2"

Pin led (LED, "P");

//CG[ board uart
#define UART_NAME UART5
constexpr uart::Config UART_CONF {
    "C12:U1,D2", UART5.ADDR, ena::UART5, 36,
};
//CG]

Dev<uart::Poll<UART_CONF>> console;

//CG[ board uart1
#define UART1_NAME USART1
#define UART1_TRIGGER(w) extern "C" { \
    void USART1_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Stream4_IRQHandler () { (w).irqDma(); } \
    void DMA1_Stream5_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART1_CONF {
    "A9:U1,A10", USART1.ADDR, ena::USART1, 72,
    DMA1.ADDR, 1-1, 4-1,5-1, 2,2,
    Irq::DMA1_Channel4, Irq::DMA1_Channel5, Irq::USART1,
};
//CG]

//CG[ board uart2
#define UART2_NAME USART2
#define UART2_TRIGGER(w) extern "C" { \
    void USART2_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Stream7_IRQHandler () { (w).irqDma(); } \
    void DMA1_Stream6_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART2_CONF {
    "A2:U1,A3", USART2.ADDR, ena::USART2, 36,
    DMA1.ADDR, 1-1, 7-1,6-1, 2,2,
    Irq::DMA1_Channel7, Irq::DMA1_Channel6, Irq::USART2,
};
//CG]

//CG[ board uart3
#define UART3_NAME USART3
#define UART3_TRIGGER(w) extern "C" { \
    void USART3_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Stream2_IRQHandler () { (w).irqDma(); } \
    void DMA1_Stream3_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART3_CONF {
    "B10:U1,B11", USART3.ADDR, ena::USART3, 36,
    DMA1.ADDR, 1-1, 2-1,3-1, 2,2,
    Irq::DMA1_Channel2, Irq::DMA1_Channel3, Irq::USART3,
};
//CG]

//CG[ board uart4
#define UART4_NAME UART4
#define UART4_TRIGGER(w) extern "C" { \
    void UART4_IRQHandler () { (w).irqIdle(); } \
    void DMA2_Stream5_IRQHandler () { (w).irqDma(); } \
    void DMA2_Stream3_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART4_CONF {
    "C10:U1,C11", UART4.ADDR, ena::UART4, 36,
    DMA2.ADDR, 2-1, 5-1,3-1, 2,2,
    Irq::DMA2_Channel5, Irq::DMA2_Channel3, Irq::UART4,
};
//CG]

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1 || fd == 2)
        console.write(buf, len);
    return len;
}

void initBoard () {
    fastClock();
    cycles::init();
    console.init(2'000'000);

    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}
