// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "g431k"

//CG1 board leds
#define LED  "B8"

//CG[ board uart
#define UART_NAME USART2
constexpr uart::Config UART_CONF {
    "A2:U7,A3", USART2.ADDR, ena::USART2, 170,
};
//CG]
