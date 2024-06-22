#include <jee.h>
#include <jee/hal.h>
#include <jee/eth.h>
#include <jee/util/net.h>
using namespace jeeh;
using namespace jeeh::net;
#include "defs.h"
#include "console.h"

extern "C" uint8_t* _sbrk (uint32_t);

constexpr Pin led (LED);  // defined in platformio.ini

int main () {
    fastClock();
    cycles::init();

    uint32_t stack [300];
    sys::init(stack); // enable multi-threading

    Uart uart ('U');
    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });
    consoleWriter = uartWriter<'U'>;

    printf("%s: ether @ %u MHz\n", SVDNAME, SystemCoreClock/1'000'000);

    const auto MHZ = SystemCoreClock/1'000'000;

    // measure the overhead of thread locks
    Lock lock;
    auto t = cycles::count();
    lock.acquire();
    lock.release();
    t = cycles::count() - t;

    printf(" [main] cycles %u lock %d ns msp %08x\n",
            cycles::count(), (1000*t)/MHZ, stack);

    if constexpr (strstr(ETHER_PINS, "A8:") != nullptr) // only on f429hy
        RCC[0x08](21,2) = 2; // CFGR: MCO1 set to HSE

    // f439n144 and f7508dk pins, all using alt mode 11:
    //      A1: refclk, A2: mdio, A7: crsdiv, C1: mdc, C4: rxd0,
    //      C5: rxd1, G11: txen, G13: txd0, B13 or G14: txd1
    Pin::config(ETHER_PINS);
    // f750d: A1:PH11,A2,A7,C1,C4,C5,G11,G13,G14
    // f429hy: A1:PH11,A2,A7,C1,C4,C5,G11,G13,G14,A8:PV0

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

    sys::wait(0); // start Ticker
    { Message m { net.mTag, 'I' }; sys::send(m); } // net.init();

#if 0
    led.mode("P");  // push-pull output
    while (true) {
        led.toggle();
        sys::wait(250);
    }
#endif

    Message m { 'U', 'R' };
    while (true) {
        printf("dump/list/reset/stats/toggle - "
#if STM32F7 || STM32H7
               "cache/uncache - "
#endif
               "Arp/Cache/Dns/Flood/Listen/Tftp ?\n");

        sys::send(m);
        while (true) {
            auto& r = sys::recv();
            if (&r == &m)
                break;
            printf("recv %p %p\n", &m, &r);
        }
        assert(m.mLen >= 1);

        switch (*m.mPtr) {
            //case 'r': printf("RESET\n"); systemReset(); break;
            //case 't': toggleCmd(); break;

            case 's': {
#if 0 // TODO
                auto t = cycles::count();
                printf("  %u wakeups: elapsed %u ms, running %u us\n",
                        stats[0], (t - stats[3])/MHZ/1000U, stats[2]/200U);
                stats[0] = stats[2] = 0;
                stats[3] = t;
#endif
                break;
            }

#if STM32F7 || STM32H7
            case 'c': cache::enable(); break;
            case 'u': cache::disable(); break;
#endif

            case 'd':
                logDump(stack, sizeof stack, "irq stack");
                break;

            case 'l': {
#if 0 // TODO
                auto unused = [](uint32_t* p) {
                    auto i = 0;
                    while (p[i] == 0xDDDD'DDDD)
                        ++i;
                    return i;
                };
                uint32_t n = Thread::dummy.mLen;
                auto mh = 0, mk = 0;
                for (auto i = 0U; i < n; ++i)
                    if (Thread::entry(i) != nullptr)
                        ++mh;
                for (auto e : Task::tasks)
                    if (e != nullptr)
                        ++mk;
                printf("C   ADDR    N  LOW/SP/TOP  -  "
                       "%d/%d threads, %d/%d tasks:\n",
                       mh, n, mk, sizeof Task::tasks / sizeof *Task::tasks);
                for (auto i = 0U; i < n; ++i) {
                    auto e = Thread::entry(i);
                    if (e != nullptr) {
                        auto bottom = (uint32_t*) (e + 1);
                        auto low = unused(bottom);
                        auto c = ' ';
                        auto sp = e->sp;
                        auto top = e->mLen;
                        if (e == Thread::current) {
                            c = '*';
                            sp = &n; // current sp, not saved sp
                        }
                        if (i == 0) {
                            bottom = (uint32_t*) _sbrk(0); // top of heap
                            top += Thread::dummy.mLen; // thread slots at top
                        }
                        printf("%c %08x %2d: %u/%u/%u",
                                c, e, i, low, sp - bottom, top);
                        auto nt = 0;
                        for (auto e : Task::tasks)
                            if (e != nullptr && e->owner == i)
                                ++nt;
                        if (nt > 0)
                            printf("  (%d tasks)", nt);
                        printf("\n");
                    }
                }
#endif
                break;
            }

            case 'A': {
                constexpr IpAddr ips [] = {
                    {192,168,178,1},
                    {192,168,178,4},
                    {192,168,178,41},
                    {192,168,178,81},
                    {192,168,111,111},
                };
                static int idx;
                auto& ip = ips[idx];
                idx = (idx + 1) % (sizeof ips / sizeof *ips);

                union { IpAddr ip {}; MacAddr mac; } req;
                req.ip = ip;

                Message m { net.mTag, 'A', (uint16_t) sizeof req, (uint8_t*) &req };
                auto t = 0;// TODO ticker.millis();
                sys::call(m);
                t = 0;// TODO ticker.millis() - t;
                assert(!m.inUse());

                SmallBuf sb;
                printf("  %s: mac %s in %u ms\n",
                        ip.asStr(), req.mac.asStr(sb), t);
                break;
            }

            case 'C':
                printf("ARP cache:\n");
                for (auto& e : ni.arpCache.items)
                    if (e.node != 0) {
                        SmallBuf sb;
                        IpAddr ip = ni.arpCache.prefix + e.node;
                        printf("  %-15s = %s\n", ip.asStr(), e.mac.asStr(sb));
                    }
                break;

            case 'D': {
                constexpr char const* domains [] = {
                    "houtje.fritz.box",
                    "touwtje.fritz.box",
                    "fritz.box",
                    "freedom.nl",
                    "touwtje",
                    "blah",
                };
                static int idx;
                auto name = domains[idx];
                idx = (idx + 1) % (sizeof domains / sizeof *domains);

                Message m { net.mTag, 'D', 0, (uint8_t*) name };
                auto t = 0;// TODO ticker.micros();
                sys::call(m);
                t = 0;// TODO ticker.micros() - t;
                assert(!m.inUse());

                IpAddr a = (uint32_t) m.mPtr;
                printf("  %s: ip %s in %u us\n", name, a.asStr(), t);
                break;
            }

            case 'F': {
                auto mp = ni.spares.pull();
                assert(mp != nullptr);
                assert(!mp->inUse());

                constexpr auto N = 500, M = 1500-8-20; // payload w/o Udp & Ip4
		memset(mp->mPtr+42, '.', M);
		mp->mPtr[M+41] = '\n';

                Message m { net.mTag, 'F', M, (uint8_t*) mp };
                uint32_t u = cycles::count();
                for (auto i = 0; i < N; ++i)
                    sys::call(m);
                u = cycles::count() - u;

                auto us = u / N / MHZ;
                printf(" %d x %u us per %db payload = %d Mbps\n",
                        N, us, M, (M * 8) / us);

		ni.addToSpares(*mp);
                break;
            }

            case 'L': {
                Message m { net.mTag, 'L' };
                sys::send(m);
                break;
            }

            case 'T': {
                auto buf = sys::pool(45'000);
                memset(buf, '.', 45'000);

                Message m { net.mTag, 'T', 45'000, buf, (intptr_t) "x" };
                auto t = 0;// TODO ticker.micros();
                sys::call(m);
                t = 0;// TODO ticker.micros() - t;
                assert(!m.inUse());

                printf("  tftp got %u b in %u us\n", m.mArg, t);
                break;
            }

            default: printf("?\n"); break;
        }
    }
}
