// Lines with "CG" control the code-generated parts of this file.

//CG1 board leds
#define LED  "B8"

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:UH7,A3"
#define UART_FREQ  170
#define UART_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,1-1,2-1,27,26
//CG]

void jeeh::fail(void const*, char const*, int) { while (true) {} }
void jeeh::LowPower::start (Message&) {}
void jeeh::LowPower::finish () {}
