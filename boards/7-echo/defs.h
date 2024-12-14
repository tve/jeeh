// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "g431k"

//CG1 board leds
#define LED  "B8"

//CG[ board uart
#define UART_NAME USART2
#define UART_TRIGGER(w) extern "C" { \
    void USART2_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Channel1_IRQHandler () { (w).irqDma(); } \
    void DMA1_Channel2_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART_CONF {
    "A2:U7,A3", USART2.ADDR, ena::USART2, 170,
    DMA1.ADDR, 1-1, 1-1,2-1, 27,26,
    Irq::DMA1_CH1, Irq::DMA1_CH2, Irq::USART2,
};
//CG]
