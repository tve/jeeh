// work around the mess in SVD w.r.t. naming inconsistencies
#define USART4 UART4

#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
#include <jee/uart.h>
using namespace jeeh;
#include "defs.h"

uart::Work<UART_TYPE> uart3 (UART_CONF);
IRQ_HANDLER(USART3, uart3.idleIrq)
IRQ_HANDLER(DMA1_Stream3, uart3.dmaIrq)
IRQ_HANDLER(DMA1_Stream1, uart3.dmaIrq)

uart::Work<UART1_TYPE> uart1 (UART1_CONF);
IRQ_HANDLER(USART1, uart1.idleIrq)
IRQ_HANDLER(DMA2_Stream7, uart1.dmaIrq)
IRQ_HANDLER(DMA2_Stream2, uart1.dmaIrq)

uart::Work<UART2_TYPE> uart2 (UART2_CONF);
IRQ_HANDLER(USART2, uart2.idleIrq)
IRQ_HANDLER(DMA1_Stream6, uart2.dmaIrq)
IRQ_HANDLER(DMA1_Stream5, uart2.dmaIrq)

uart::Work<UART4_TYPE> uart4 (UART4_CONF);
IRQ_HANDLER(UART4, uart4.idleIrq)
IRQ_HANDLER(DMA1_Stream4, uart4.dmaIrq)
IRQ_HANDLER(DMA1_Stream2, uart4.dmaIrq)

uart::Work<UART5_TYPE> uart5 (UART5_CONF);
IRQ_HANDLER(UART5, uart5.idleIrq)
IRQ_HANDLER(DMA1_Stream7, uart5.dmaIrq)
IRQ_HANDLER(DMA1_Stream0, uart5.dmaIrq)

uart::Work<UART6_TYPE> uart6 (UART6_CONF);
IRQ_HANDLER(USART6, uart6.idleIrq)
IRQ_HANDLER(DMA2_Stream6, uart6.dmaIrq)
IRQ_HANDLER(DMA2_Stream1, uart6.dmaIrq)

uart::Sync<UART9_TYPE> uart9 (UART9_CONF);
//IRQ_HANDLER(UART9, uart9.idleIrq)
//IRQ_HANDLER(DMA2_Stream0, uart9.dmaIrq)
//IRQ_HANDLER(DMA2_Stream7, uart9.dmaIrq) // conflicts with uart1 tx

uart::Work<UART10_TYPE> uart10 (UART10_CONF);
IRQ_HANDLER(USART10, uart10.idleIrq)
IRQ_HANDLER(DMA2_Stream5, uart10.dmaIrq)
IRQ_HANDLER(DMA2_Stream3, uart10.dmaIrq)

struct Matrix : Worker {
    enum TAG { START, R1, R2, R3, R4, R5, R6, R10, T1, T2, T3, T4, T5, T6, NE };
    static constexpr char const* names [] = {
        " S","R1","R2","R3","R4","R5","R6","R10","T1","T2","T3","T4","T5","T6"
    };

    enum { NR=200 };
    uint8_t buf1 [NR], buf2 [NR], buf3 [NR], buf4 [NR], buf5 [NR], buf6 [NR], buf10 [NR];
    uint32_t counts [NE] ={}, bytes [NE] ={};
    bool verbose =true;

    Event process (Event in, Event out, void*) {
        auto tag = in.eTag;
        ++counts[tag];
        bytes[tag] += in.eVal;
        if (verbose && tag < T1)
            logf("%s %d", names[tag], in.eVal);
        switch (tag) {
            case START: // issue read requests on all UARTs
                uart1.read(0, { wId, R1 });
                uart2.read(0, { wId, R2 });
                uart3.read(0, { wId, R3 });
                uart4.read(0, { wId, R4 });
                uart5.read(0, { wId, R5 });
                uart6.read(0, { wId, R6 });
                //uart10.read(0, { wId, R10 });
                break;
            case R1:
                memcpy(buf1, uart1.rxPtr, in.eVal); // keep copy of recv'd data
                uart1.read(in.eVal, { wId, R1 }); // consume and start new read
                uart1.write(buf1, in.eVal, { wId, T1 }); // send data out again
                break;
            case R2:
                memcpy(buf2, uart2.rxPtr, in.eVal);
                uart2.read(in.eVal, { wId, R2 });
                uart2.write(buf2, in.eVal, { wId, T2 });
                break;
            case R3: // console input
                memcpy(buf3, uart3.rxPtr, in.eVal);
                uart3.read(in.eVal, { wId, R3 });
                switch (buf3[0]) {
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
                memcpy(buf4, uart4.rxPtr, in.eVal);
                uart4.read(in.eVal, { wId, R4 });
                uart4.write(buf4, in.eVal, { wId, T4 });
                break;
            case R5:
                memcpy(buf5, uart5.rxPtr, in.eVal);
                uart5.read(in.eVal, { wId, R5 });
                uart5.write(buf5, in.eVal, { wId, T5 });
                break;
            case R6:
                memcpy(buf6, uart6.rxPtr, in.eVal);
                uart6.read(in.eVal, { wId, R6 });
                uart6.write(buf6, in.eVal, { wId, T6 });
                break;
            case R10:
                memcpy(buf10, uart10.rxPtr, in.eVal);
                uart10.read(in.eVal, { wId, R10 });
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

int main () {
    initBoard();
    logf("scb %x %x", SCB.byte(0x1F), SCB.byte(0x22));
    SCB.byte(0x1F) = 0xFF; // SVC prio
    SCB.byte(0x22) = 0xFF; // PendSV prio

    uart1.init(UART1_PINS, 1'000'000);    // tx: B6  rx: B3  < D15 #9
    uart2.init(UART2_PINS, 1'000'000);    // tx: A2  rx: A3  < B6  #1
    uart3.init(UART_PINS, 2'000'000); // TODO probably 8/16 MHz xtal mixup
    uart4.init(UART4_PINS, 1'000'000);    // tx: A0  rx: C11 < A2  #2
    uart5.init(UART5_PINS, 1'000'000);    // tx: C12 rx: D2  < A0  #4
    uart6.init(UART6_PINS, 1'000'000);    // tx: G14 rx: G9  < C12 #5
    uart9.init(UART9_PINS, 1'000'000);    // tx: D15 rx: D14
    uart10.init(UART10_PINS, 1'000'000);  // tx: E3  rx: E2  < G14 #6

    auto s = " abcdefghijklmnopqrstuvwxyz + ABCDEFGHIJKLMNOPQRSTUVWXYZ /\n";
    auto n = strlen(s);

    Matrix matrix;
    auto mxId = matrix.init();
    Worker::send({ mxId, matrix.START });

    while (true) {
        //auto start = cycles::count();
        uart9.transfer(true, (uint8_t*) s, n);
        //logf("%d cy", cycles::count()-start);

        led = 1;
        cycles::usBusy(10000);
        led = 0;
        //cycles::msBusy(50);
    }
}
