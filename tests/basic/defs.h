//CG2 board exti
#define EXT_IN  "A10"
#define EXT_OUT "A9"

//CG1 board leds
#define LED  "B13"

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:7,A10"
#define UART_FREQ  80
#define UART_CONF  Irq::DMA1_Channel4,Irq::DMA1_Channel5,1-1,4-1,5-1,2,2
//CG]
