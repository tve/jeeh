// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "l475d"

//CG1 board leds
#define LED  "A5"

//CG: board uart

Pin led (LED, "P");

uart::Poll<UART_NAME.ADDR> console (ena::UART_NAME, UART_FREQ);

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
