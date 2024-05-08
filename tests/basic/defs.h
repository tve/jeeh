//CG2 board exti
#define EXT_IN  "A1"
#define EXT_OUT "A0"

//CG1 board leds
#define LED  "A5"

//CG[ board uart
#define UART_NAME  UART4
#define UART_PINS  "A0:8,A1"
#define UART_FREQ  80
#define UART_CONF  Irq::DMA2_Channel3,Irq::DMA2_Channel5,2-1,3-1,5-1,2,2
//CG]
