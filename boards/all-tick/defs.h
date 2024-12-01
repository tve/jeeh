// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "f303z"

//CG[ board leds
#define LED  "B0"
#define LED1 "B0"
#define LED2 "B7"
#define LED3 "B14"
//CG]

//CG[ board uart
#define UART_NAME  USART3
#define UART_PINS  "D8:7,D9"
#define UART_FREQ  36
#define UART_TYPE  USART3.ADDR, DMA1.ADDR, 3-0, 1-0
#define UART_CONF  { ena::USART3, 36, Irq::USART3, \
                     Irq::DMA1_CH3, Irq::DMA1_CH1, { 1-1,4,4 } }
#define UART_TRIGGER(w) extern "C" { \
    void USART3_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Channel3_IRQHandler () { (w).irqDma(); } \
    void DMA1_Channel1_IRQHandler () { (w).irqDma(); } \
}
//CG]

Pin led (LED, "P");

#ifdef UART_TYPE
uart::Sync<UART_TYPE> console (UART_CONF);
#else
uart::Poll<UART_NAME.ADDR> console (ena::UART_NAME, UART_FREQ);
#endif

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1 || fd == 2)
        console.transfer(true, (uint8_t*) buf, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(2, (char*) ptr, len);
}

void initBoard () {
    fastClock();
    led.toggle();
    cycles::init();
    console.init(UART_PINS, 2'000'000);

    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}
