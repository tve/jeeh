// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f411bp"

//CG1 board leds
#define LED  "C13"

//CG[ board uart
#define UART_NAME USART1
#define UART_TRIGGER(w) extern "C" { \
    void DMA2_Stream7_IRQHandler () { (w).irqTx(); } \
    void DMA2_Stream5_IRQHandler () { (w).irqRx(); } \
    void USART1_IRQHandler () { (w).irqRx(); } \
}
constexpr uart::Config UART_CONF {
    "A9:U7,A10", USART1.ADDR, ena::USART1, 100,
    DMA2.ADDR, 2-1, 7-0,5-0, 4,4,
    Irq::DMA2_Stream7, Irq::DMA2_Stream5, Irq::USART1, 64,
};
//CG]
