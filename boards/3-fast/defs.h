// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f303o"

//CG3 board leds
#define LED  "A5"
#define LED1 "A5"
#define LED2 "A1"

//CG[ board uart
#define UART_NAME USART3
constexpr uart::Config UART_CONF {
    "B10:7,B11", USART3.ADDR, ena::USART3, 36,
};
//CG]
