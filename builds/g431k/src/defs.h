//CG2 board exti
#define EXT_IN  "A10"
#define EXT_OUT "A9"

//CG1 board leds
#define LED  "B8"

//CG1 board serio
#define SERIO_PINS "A9:7,A10"

//CG[ board uart
#define UART_PINS  "A9:7,A10"
#define UART_NAME  USART1
#define UART_FREQ  150
#define UART_VERS  2
#define UART_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,2-0,1-0,24,25
//CG]
