// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f303k"

//CG1 board leds
#define LED  "B3"

//CG[ board uart
#define UART_NAME USART2
#define UART_TRIGGER(w) extern "C" { \
    void DMA1_Channel7_IRQHandler () { (w).irqTx(); } \
    void DMA1_Channel6_IRQHandler () { (w).irqRx(); } \
    void USART2_IRQHandler () { (w).irqRx(); } \
}
constexpr uart::Config UART_CONF {
    "A2:7,A15", USART2.ADDR, ena::USART2, 36,
    DMA1.ADDR, 1-1, 7-1,6-1, 2,2,
    Irq::DMA1_CH7, Irq::DMA1_CH6, Irq::USART2, 64,
};
//CG]
