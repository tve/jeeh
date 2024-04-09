//CG2 board exti
#define EXT_IN  "G14"
#define EXT_OUT "G9"

//CG[ board leds
#define LED  "B0"
#define LED1 "B0"
#define LED2 "B7"
#define LED3 "B14"
//CG]

//CG[ board uart
#define UART_NAME  USART3
#define UART_PINS  "D8:7,D9"
#define UART_FREQ  45
#define UART_CONF  Irq::DMA1_Stream3,Irq::DMA1_Stream1,1-1,3-0,1-0,4,4
//CG]
