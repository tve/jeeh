// Lines with "CG" control the code-generated parts of this file.

//CG1 board leds
#define LED  "B3"

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:7,A15:3"
#define UART_FREQ  80
#define UART_CONF  Irq::DMA1_CH7,Irq::DMA1_CH6,1-1,7-1,6-1,2,2
//CG]

void jeeh::fail(void const*, char const*, int) {
    constexpr Pin led (LED);
    led.mode("P");
    while (true) {
        led.toggle();
        sys::wait(100);
    }
}
