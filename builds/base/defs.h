// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "g431k"

//CG1 board leds
#define LED  "B8"

const Pin led (LED,"P");

//CG[ board uart2
#define UART2_NAME  USART1
#define UART2_PINS  "A9:7,A10"
#define UART2_FREQ  170
#define UART2_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,1-1,2-1,25,24
//CG]
