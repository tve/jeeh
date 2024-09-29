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

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "B5:H5,B4,B3,A11:HP"
#define SPI_FREQ  170
#define SPI_TYPE  SPI1.ADDR,DMA1.ADDR,3-1,4-1
#define SPI_CONF  {ena::SPI1,170,Irq::DMA1_CH3,Irq::DMA1_CH4}, {1-1,11,10}
//CG]
