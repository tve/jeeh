//CG2 board exti
#define EXT_IN  "A3"
#define EXT_OUT "A2"

//CG1 board leds
#define LED  "A5"

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:7,A3"
#define UART_FREQ  108
#define UART_CONF  Irq::DMA1_Stream6,Irq::DMA1_Stream5,1-1,6-0,5-0,4,4
//CG]
