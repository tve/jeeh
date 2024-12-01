// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "g431k"

//CG1 board leds
#define LED  "B8"

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:U7,A3"
#define UART_FREQ  170
#define UART_TYPE  USART2.ADDR, DMA1.ADDR, 1-1, 2-1
#define UART_CONF  { ena::USART2, 170, Irq::USART2, \
                     Irq::DMA1_CH1, Irq::DMA1_CH2, { 1-1,27,26 } }
#define UART_TRIGGER(w) extern "C" { \
    void USART2_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Channel1_IRQHandler () { (w).irqDma(); } \
    void DMA1_Channel2_IRQHandler () { (w).irqDma(); } \
}
//CG]
