// work around the mess in SVD w.r.t. naming inconsistencies
#define USART4 UART4

#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
#include <jee/uart.h>
using namespace jeeh;
#include "defs.h"

Dev<uart::Async<UART_CONF>> uart3;
UART_TRIGGER(uart3)

Dev<uart::Async<UART1_CONF>> uart1;
UART1_TRIGGER(uart1)

Dev<uart::Async<UART2_CONF>> uart2;
UART2_TRIGGER(uart2)

Dev<uart::Async<UART4_CONF>> uart4;
UART4_TRIGGER(uart4)

Dev<uart::Async<UART5_CONF>> uart5;
UART5_TRIGGER(uart5)

Dev<uart::Async<UART6_CONF>> uart6;
UART6_TRIGGER(uart6)

Dev<uart::Sync<UART9_CONF>> uart9;
//UART9_TRIGGER(uart9)

Dev<uart::Async<UART10_CONF>> uart10;
#define UART10_IRQHandler USART10_IRQHandler 
UART10_TRIGGER(uart10) // TODO wrong code: UART10... iso USART10_IRQHandler !

struct Matrix : Task {
    enum TAG { START, R1, R2, R3, R4, R5, R6, R10, T1, T2, T3, T4, T5, T6, NE };
    static constexpr char const* names [] = {
        " S","R1","R2","R3","R4","R5","R6","R10","T1","T2","T3","T4","T5","T6"
    };

    enum { NR=200 };
    uint8_t bufs [T1][NR];
    uint32_t counts [NE] ={}, bytes [NE] ={};
    bool verbose =true;

    template< typename T >
    void reader (T& u, uint16_t n, TAG t) {
        u.setReply({ tId, t });
        u.read(nullptr, n);
    }

    template< typename T >
    void writer (T& u, uint16_t n, TAG r, TAG t) {
        u.setReply({ tId, t });
        u.write(bufs[r], n);
    }

    template< typename T >
    void copier (T& u, uint16_t n, TAG r, TAG t) {
        assert(n <= NR);
        memcpy(bufs[r], u.rxPtr, n); // keep copy of recv'd data
        reader(u, n, r); // consume and start new read
        writer(u, n, r, t); // send data out again
    }

    Event process (Event in, Event out) {
        auto tag = in.eTag;
        ++counts[tag];
        bytes[tag] += in.eVal;
        if (verbose && tag < T1)
            logf("%s %d", names[tag], in.eVal);
        switch (tag) {
            case START: // issue read requests on all UARTs
                reader(uart1, 0, R1);
                reader(uart2, 0, R2);
                reader(uart3, 0, R3);
                reader(uart4, 0, R4);
                reader(uart5, 0, R5);
                reader(uart6, 0, R6);
                //reader(uart10, 0, R10);
                break;
            case R1:
                copier(uart1, in.eVal, R1, T1);
                break;
            case R2:
                copier(uart2, in.eVal, R2, T2);
                break;
            case R3: // console input
                memcpy(bufs[R3], uart3.rxPtr, in.eVal);
                reader(uart3, in.eVal, R3);
                switch (bufs[R3][0]) {
                    case 'q': verbose = false; break;
                    case 'v': verbose = true; break;
                    case 's': showStats(); break;
                    case 'c': for (auto i = 0; i < NE; ++i)
                                logf("%3s %8d %8d",
                                        names[i], counts[i], bytes[i]);
                              break;
                }
                break;
            case R4:
                copier(uart4, in.eVal, R4, T4);
                break;
            case R5:
                copier(uart5, in.eVal, R5, T5);
                break;
            case R6:
                copier(uart6, in.eVal, R6, T6);
                break;
            case R10:
                memcpy(bufs[R10], uart10.rxPtr, in.eVal);
                reader(uart10, in.eVal, R10);
                break;
            case T1:
            case T2:
            case T4:
            case T5:
            case T6:
                if (verbose)
                    logf("%s %d", names[tag], in.eVal); // transmission completed
                break;
            default:
                fail();
        }
        return out;
    }
};

Matrix matrix;

int main () {
    initBoard();

    uart1.init(1'000'000);  // tx: B6  rx: B3  < D15 #9
    uart2.init(1'000'000);  // tx: A2  rx: A3  < B6  #1
    uart3.init(1'000'000);  // tx: D8  rx: D9
    uart4.init(1'000'000);  // tx: A0  rx: C11 < A2  #2
    uart5.init(1'000'000);  // tx: C12 rx: D2  < A0  #4
    uart6.init(1'000'000);  // tx: G14 rx: G9  < C12 #5
    uart9.init(1'000'000);  // tx: D15 rx: D14
    uart10.init(1'000'000); // tx: E3  rx: E2  < G14 #6

    matrix.init();

    auto s = " abcdefghijklmnopqrstuvwxyz + ABCDEFGHIJKLMNOPQRSTUVWXYZ /\n";
    auto n = strlen(s);

    while (true) {
        //auto start = cycles::count();
        uart9.write(s, n);
        //logf("%d cy", cycles::count()-start);

        led = 1;
        cycles::usBusy(10000);
        led = 0;
        //cycles::msBusy(50);
    }
}
