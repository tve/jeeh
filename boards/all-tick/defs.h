// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f303k"

//CG1 board leds
#define LED  "B3"

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:7,A3"
#define UART_FREQ  36
#define UART_TYPE  USART2.ADDR, DMA1.ADDR, 7-1, 6-1
#define UART_CONF  { ena::USART2, 36, Irq::USART2, \
                     Irq::DMA1_CH7, Irq::DMA1_CH6, { 1-1,2,2 } }
#define UART_TRIGGER(w) extern "C" { \
    void USART2_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Channel7_IRQHandler () { (w).irqDma(); } \
    void DMA1_Channel6_IRQHandler () { (w).irqDma(); } \
}
//CG]

Pin led (LED, "P");

uart::Poll<UART_NAME.ADDR> console (ena::UART_NAME, UART_FREQ);
//uart::Sync<UART_TYPE> console (UART_CONF);

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        console.transfer(true, (uint8_t*) buf, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    console.transfer(true, (uint8_t*) ptr, len);
}

void initBoard () {
    fastClock();
    cycles::init();
    console.init(UART_PINS, 2'000'000);

    logf("\n%s: %s @ %d MHz",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}
