// Check the built-in real-time clock and backup registers.

#include <jee.h>
using namespace jeeh;
#include "test.h"

struct ClockDev : Device, Chain {
    enum { NONE, RISE, FALL, BOTH };

    ClockDev () : Device ('C') {}

    void init () {
        RCC(ena::SYSCFG, 1) = 1;
        EXTI[0x00](20) = 1; // EM20 in IMR1
        EXTI[0x08](20) = 1; // RT20 in RTSR1

        irqInstall((int) Irq::RTC_WKUP);
    }

    void start (Message& m) override {
        switch (m.mTag) {
            case 'T':
                append(m); // TODO wrong, should insert and adjust the delays
                startWakeup(m.mLen);
                break;
            default:
                m.mTag = -1;
                reply(&m);
        }
    }

    void finish () override {
        reply(pull());
        auto mp = first();
        if (mp != nullptr)
            startWakeup(mp->mLen);
    }

    void startWakeup (uint16_t ms) {
        assert(ms <= 16'000);
        auto sel = 3;
        auto count = (1000*ms) / 61;
        while (count >= 32768) {
            --sel;
            count /= 2;
        }

        RTC[WPR] = 0xCA; // disable write protection
        RTC[WPR] = 0x53;

        RTC[CR](10) = 0; // ~WUTE
        while (RTC[ICSR](2) == 0) {} // wait for WUTWF
        RTC[WUTR] = count;
        RTC[CR](0,3) = sel;
        RTC[CR](14) = 1; // WUTIE
        RTC[CR](10) = 1; // WUTE

        RTC[WPR] = 0xFF; // re-enable write protection TODO why?
    }

    bool interrupt (int) override {
        EXTI[0x14] = 1<<20;
        RTC[SCR] = 1<<2; // CWUTF
        RTC[CR](10) = 0; // ~WUTE
        return true;
    }

    enum { ICSR=0x0C,WUTR=0x14,CR=0x18,WPR=0x24,SCR=0x5C };
};

int main () {
    Tester t;

#if STM32G431xx
    // there are no OSC32 pins on Nucleo-32's G431KB, must use the 32 kHz LSI
    rtc::init(false);
#else
    rtc::init();
#endif

    ClockDev clock;
    clock.init();

    auto t1 = rtc::getSecs();
    logf("1: %d", t1);

    Message m { clock.dId, 'T', 300 };
    for (auto i = 0; i < 7; ++i) {
        sys::call(m);
        itmWrite(".", 1); // send some output to avoid a timeout
    }
    itmWrite("\n", 1);

    auto t2 = rtc::getSecs();
    logf("2: %d", t2);

    assert(t2 == t1 + 2);
}
