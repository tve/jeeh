// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f412d"

//CG1 board leds
#define LED  "E0"

//CG[ board uart
#define UART_NAME USART2
constexpr uart::Config UART_CONF {
    "A2:U7,A3", USART2.ADDR, ena::USART2, 50,
};
//CG]
