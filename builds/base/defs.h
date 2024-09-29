// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "g431k"

//CG1 board leds
#define LED  "B8"

const Pin led (LED,"P");

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:U7,A10"
#define UART_FREQ  170
#define UART_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,1-1,2-1,25,24
//CG]
