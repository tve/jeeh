// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f750d"

//CG1 board leds
#define LED  "I1"

//CG[ board uart
#define UART_NAME USART1
constexpr uart::Config UART_CONF {
    "A9:7,B7", USART1.ADDR, ena::USART1, 108,
};
//CG]
