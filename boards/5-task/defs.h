// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f412d"

//CG1 board leds
#define LED  "E0"

//CG[ board uart
#define UART_NAME USART2
#define UART_TRIGGER(w) extern "C" { \
    void DMA1_Stream6_IRQHandler () { (w).irqTx(); } \
    void DMA1_Stream5_IRQHandler () { (w).irqRx(); } \
    void USART2_IRQHandler () { (w).irqRx(); } \
}
constexpr uart::Config UART_CONF {
    "A2:U7,A3", USART2.ADDR, ena::USART2, 50,
    DMA1.ADDR, 1-1, 6-0,5-0, 4,4,
    Irq::DMA1_Stream6, Irq::DMA1_Stream5, Irq::USART2, 64,
};
//CG]
