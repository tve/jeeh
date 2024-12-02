// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "l432k"

//CG1 board leds
#define LED  "B3"

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:7,A15:3"
#define UART_FREQ  80
#define UART_TYPE  USART2.ADDR, DMA1.ADDR, 7-1, 6-1
#define UART_CONF  { ena::USART2, 80, Irq::USART2, \
                     Irq::DMA1_CH7, Irq::DMA1_CH6, { 1-1,2,2 } }
#define UART_TRIGGER(w) extern "C" { \
    void USART2_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Channel7_IRQHandler () { (w).irqDma(); } \
    void DMA1_Channel6_IRQHandler () { (w).irqDma(); } \
}
//CG]
