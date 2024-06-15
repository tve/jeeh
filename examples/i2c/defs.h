// Lines with "CG" control the code-generated parts of this file.

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:7,A15:3"
#define UART_FREQ  80
#define UART_CONF  Irq::DMA1_CH7,Irq::DMA1_CH6,1-1,7-1,6-1,2,2
//CG]

//CG3 board i2c
#define I2C_NAME  I2C1
#define I2C_PINS  "A2:7,A15:3"
#define I2C_CONF  Irq::I2C1_EV,1-1,7-1,6-1,2,2
