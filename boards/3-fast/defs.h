// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f103r"

//CG1 board leds
#define LED  "A5"

//CG[ board uart
#define UART_NAME USART2
constexpr uart::Config UART_CONF {
    "A2:U1,A3", USART2.ADDR, ena::USART2, 36,
};
//CG]
