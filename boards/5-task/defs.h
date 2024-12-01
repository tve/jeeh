// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "l011k"

//CG1 board leds
#define LED  "B3"

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:U4,A3"
#define UART_FREQ  32
#define UART_TYPE  USART2.ADDR, DMA1.ADDR, 4-1, 5-1
#define UART_CONF  { ena::USART2, 32, Irq::USART2, \
                     Irq::DMA1_Channel4, Irq::DMA1_Channel5, { 1-1,4,4 } }
#define UART_TRIGGER(w) extern "C" { \
    void USART2_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Stream4_IRQHandler () { (w).irqDma(); } \
    void DMA1_Stream5_IRQHandler () { (w).irqDma(); } \
}
//CG]
