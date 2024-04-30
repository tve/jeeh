// Lines with "CG" control the code-generated parts of this file.

//CG[ board leds
#define LED  "B0"
#define LED1 "B0"
#define LED2 "B1"
#define LED3 "C13"
#define LED4 "A4"
//CG]

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:7,A10"
#define UART_FREQ  90
#define UART_CONF  Irq::DMA2_Stream7,Irq::DMA2_Stream5,2-1,7-0,5-0,4,4
//CG]
