#include <jee.h>
using namespace jeeh;
#include "defs.h"

namespace jeeh {
    void fail (char const*, int) {
        itmWrite("FAIL\n", 5);
        while (true) {}
    }
} // namespace jeeh

int main () {
    extern uint32_t g_pfnVectors [];
    *(uint32_t**) 0xE000'ED08 = g_pfnVectors; // fix SCB->VTOR if it's in RAM

    Pin led (LED);
    led.mode("P");

    assert(led == 0);
    led = 1;
    assert(led == 1);

    //for (auto i = 0; i < 100'000; ++i) asm ("");

    itmWrite("TEST\n", 5);
    itmWrite("OK\n", 3);
}
