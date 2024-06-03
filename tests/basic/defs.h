//CG2 board exti
#define EXTI_IN "A10"
#define EXTI_OUT "A9"

//CG1 board leds
#define LED  "B8"

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:7,A10"
#define UART_FREQ  170
#define UART_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,1-1,2-1,25,24
//CG]
