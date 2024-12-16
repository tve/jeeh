// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f411bp"

//CG1 board leds
#define LED  "C13"

//CG[ board uart
#define UART_NAME USART1
constexpr uart::Config UART_CONF {
    "A9:U7,A10", USART1.ADDR, ena::USART1, 100,
};
//CG]
