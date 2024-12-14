// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f723d"

//CG1 board leds
#define LED  "B1"

//CG[ board uart
#define UART_NAME USART6
#define UART_TRIGGER(w) extern "C" { \
    void USART6_IRQHandler () { (w).irqIdle(); } \
    void DMA2_Stream6_IRQHandler () { (w).irqDma(); } \
    void DMA2_Stream2_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART_CONF {
    "C6:8,C7", USART6.ADDR, ena::USART6, 108,
    DMA2.ADDR, 2-1, 6-0,2-0, 5,5,
    Irq::DMA2_Stream6, Irq::DMA2_Stream2, Irq::USART6, 64,
};
//CG]
