#include <jee.h>
#include <jee/hal.h>
#include <jee/eth.h>
#include <jee/net.h>
using namespace jeeh;
using namespace jeeh::net;
#include "defs.h"
#include "console.h"

extern "C" uint8_t* _sbrk (uint32_t);

constexpr Pin led (LED);  // defined in platformio.ini

int main () {
    hardFaulter = hardFaultHandler;
    fastClock();

    uint32_t stack [300];
    sys::init(stack); // enable multi-threading

    Uart uart ('U');
    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });
    consoleWriter = uartWriter<'U'>;

    printf("%s: ether @ %u MHz\n", SVDNAME, SystemCoreClock/1'000'000);

    // f439n144 and f7508dk pins, all using alt mode 11:
    //      A1: refclk, A2: mdio, A7: crsdiv, C1: mdc, C4: rxd0,
    //      C5: rxd1, G11: txen, G13: txd0, B13 or G14: txd1
    Pin::config(ETHER_PINS);
    // f750d: A1:PH11,A2,A7,C1,C4,C5,G11,G13,G14

    MacAddr mac {0x32,0x31,0xC4,0x8E,0x32,0x66}; //0x?[26AE]:* is local

    Eth eth ('E', 8);
    eth.init(mac.b);

    Interface ni (mac, 'E', 14'000);

    extern uint8_t _sdata [], _ebss [], _estack [];
    auto heapTop = _sbrk(0);
    auto currSp = (uint8_t*) &heapTop;
    printf("  [mem] data %08x (%u b) heap %08x (%u b)\n"
           "        free %08x (%u kb) stack %08x (%u b) end %08x\n",
                _sdata, _ebss-_sdata, _ebss, heapTop-_ebss, heapTop,
                (currSp-heapTop)/1024, currSp, _estack-currSp, _estack);

    Worker net (ni);
    printf("  [net] mac %s eth %d net %d\n", ni.mac.asStr(), ni.drv, net.mTag);
    //net.init();

    led.mode("P");  // push-pull output
    while (true) {
        led.toggle();
        sys::wait(250);
    }
}
