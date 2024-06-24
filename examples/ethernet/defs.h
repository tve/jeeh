// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f429hy"

//CG1 board ether
#define ETHER_PINS "A1:PH11,A2,A7,C1,C4,C5,G11,G13,G14,A8:PV0"

//CG[ board leds
#define LED  "B0"
#define LED1 "B0"
#define LED2 "B1"
#define LED3 "C13"
//CG]

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:7,A10"
#define UART_FREQ  90
#define UART_CONF  Irq::DMA2_Stream7,Irq::DMA2_Stream5,2-1,7-0,5-0,4,4
//CG]
