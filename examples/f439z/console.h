#include <cstdio>

void (*consoleWriter) (void const*, size_t) = nullptr;

void jeeh::hardFaultHandler (uint32_t* sp) {
    enum { CFSR=0x28, HFSR=0x2C, MMAR=0x34, BFAR=0x38 };

    uint32_t hfsr = SCB[HFSR], cfsr = SCB[CFSR],
            bfar = SCB[BFAR], mmar = SCB[MMAR];

    asm ("cpsid i"); // disable all interrupts
    consoleWriter = nullptr; // can't use IRQs once there's a hardfault

    printf("\n[Hard Fault]  SP=%08x  HFSR=%08x  CFSR=%08x\n", sp, hfsr, cfsr);
    if (hfsr & (1<<30)) {
        if (cfsr & 0xFFFF0000)
            printf("  Usage fault %04x\n", cfsr >> 16);
        if (cfsr & 0xFF00) {
            printf("  Bus fault %02x\n", (uint8_t) (cfsr >> 8));
            if (cfsr & (1<<15))
                printf("    BFAR %08x\n", bfar);
        }
        if (cfsr & 0xFF) {
            printf("  Memory fault %02x\n", (uint8_t) cfsr);
            if (cfsr & (1<<7))
                printf("    MMAR %08x\n", mmar);
        }
    }

    printf("\t R0=%08x  R1=%08x  R2=%08x  R3=%08x\n",
            sp[0], sp[1], sp[2], sp[3]);
    printf("\tR12=%08x  LR=%08x  PC=%08x PSR=%08x\n",
            sp[4], sp[5], sp[6], sp[7]);

    fail();
}

void jeeh::fail (void const* a, char const* f, int n) {
    printf("failed at %s:%d\nfailed caller: %p\n", f, n, a);
    while (true) {}
}

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
