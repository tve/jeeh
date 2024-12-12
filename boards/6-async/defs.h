// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f446z"

//CG[ board leds
#define LED  "B0"
#define LED1 "B0"
#define LED2 "B7"
#define LED3 "B14"
//CG]

//CG[ board uart
#define UART_NAME USART3
#define UART_TRIGGER(w) extern "C" { \
    void USART3_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Stream3_IRQHandler () { (w).irqDma(); } \
    void DMA1_Stream1_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART_CONF {
    "D8:7,D9", USART3.ADDR, ena::USART3, 90,
    DMA1.ADDR, 1-1, 3-0,1-0, 4,4,
    Irq::DMA1_Stream3, Irq::DMA1_Stream1, Irq::USART3,
};
//CG]
