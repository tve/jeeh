// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f723d"

//CG1 board leds
#define LED  "B1"

//CG[ board uart
#define UART_NAME USART6
constexpr uart::Config UART_CONF {
    "C6:8,C7", USART6.ADDR, ena::USART6, 108,
};
//CG]
