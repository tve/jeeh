#include <cstdio>

void (*consoleWriter) (void const*, size_t) = nullptr;

void polledWriter (void const* ptr, size_t len) {
    // assume the UART has already been set up, e.g. by its DMA driver
    enum { ISR=0x00, TDR=0x04 };
    for (auto i = 0U; i < len; ++i) {
        while (!UART_NAME[ISR](7)) {} // TXFNF
        UART_NAME[TDR] = ((uint8_t const*) ptr)[i];
    }
}

template< char ID >
void uartWriter (void const* ptr, size_t len) {
    Message m { ID, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
}

extern "C" int _write (int, char* ptr, int len) {
    if (consoleWriter == nullptr)
        consoleWriter = polledWriter;
    consoleWriter(ptr, len);
    return len;
}
