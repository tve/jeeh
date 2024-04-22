// This code can be compiled with either "arduino" or "cmsis" as PIO framework.
// I.e. "pio run -e l432k-arduino" or "pio run -e l432-cmsis".

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#if ARDUINO //------------------------------------------------------------------
// the Arduino header only works when included after the JeeH stuff (TODO why?)
#include <Arduino.h>
//sys::wait(100); XXX can't be used, pulls in SysTick_Handler
#define DELAY(ms) delay(ms)
#else
#define DELAY(ms) sys::wait(ms)
#endif //-----------------------------------------------------------------------

Pin led ("B3"); // Nucleo-L432KC

void setup() {
#if ARDUINO
    Serial.begin(115'200);
#endif
    printf("???\r%s @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);
    led.mode("P"); // push-pull output
}

void loop() {
    DELAY(100);
    led.toggle();

    // force an assertion failure after a few blinks
    static int count; ++count; assert(count <= 10);
}

void jeeh::fail(void const* a, char const* f, int n) {
    printf("\nfailed at %s:%d\nfailed caller: %p\n", f, n, a);
    while (true) {}
}

void jeeh::hardFaultHandler (uint32_t* sp) {
    printf("\nhard fault, sp = %p", sp);
    fail();
}

#if !ARDUINO //-----------------------------------------------------------------
// when not using the Arduino runtime, all of JeeH becomes available for use

Uart uart ('U'); // DMA/IRQ-based UART device driver, id = 'U'

extern "C" int _write (int, char* ptr, int len) {
    Message m { 'U', 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

int main () {
    hardFaulter = hardFaultHandler;
    fastClock();

    // the UART config comes from platformio.ini and is defined in defs.h
    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });
    setup();
    while (true)
        loop();
}

#endif // !ARDUINO //-----------------------------------------------------------
