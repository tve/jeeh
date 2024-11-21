#include <jee.h>
using namespace jeeh;

namespace serio {
    enum { SR=0x00, DR=0x04, BRR=0x08, CR1=0x0C };

    void init () {
        Pin::config("A9:U7");
        RCC(ena::USART1,1) = 1;
        USART1[BRR] = SystemCoreClock / 9600;
        USART1[CR1] = (1<<13) | (1<<3); // UE TE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!USART1[SR](7)) {} // TXE
            USART1[DR] = ((uint8_t const*) ptr)[i];
        }
    }
}

int main () {
    serio::init();
    serio::write("Hello, World!\n", 14);
}
