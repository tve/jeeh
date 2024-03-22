#include "jee.h"

#if STM32

namespace jeeh {

#if STM32F1
#include "stm32f1.h"
#elif STM32F3
#include "stm32f3.h"
#elif STM32F4
#include "stm32f4.h"
#elif STM32F7
#include "stm32f7.h"
#elif STM32G0
#include "stm32g0.h"
#elif STM32G4
#include "stm32g4.h"
#elif STM32H7
#include "stm32h7.h"
#elif STM32L0
#include "stm32l0.h"
#elif STM32L4
#include "stm32l4.h"
#endif // STM32??

} // namespace jeeh

using namespace jeeh;

//------------------------------------------------------------------------ ITM

#if !STM32G0 && !STM32L0 // Cortex M0+ doesn't support ITM

void jeeh::itmWrite (void const* ptr, size_t len) {
    constexpr IoReg<0xE000'0000> ITM;
    enum { TER=0xE00, TCR=0xE80 };

    if (ITM[TCR](0) && ITM[TER](0)) { // ITM and channel 0 both enabled
        auto pos = (uintptr_t) ptr;
        while (len > 0) {
            while (!ITM[0](0)) {}
            int step = pos % 4 == 0 && len >= 4 ? 4 : 1;
            if (step == 4)
                ITM[0] = *(uint32_t const*) pos;
            else
                ITM.byte(0) = *(uint8_t const*) pos;
            pos += step;
            len -= step;
        }
    }
}

#endif // !STM32L0

//--------------------------------------------------------------------- Ticker

inline namespace {

struct Ticker : Device, Chain {
    volatile uint32_t ticks;
    uint16_t rate;

    Ticker () : Device ('@'), ticks (0), rate (0) {
        SCB.byte(0x23) = 0xFF; // irq #15: lowest IRQ priority
    }

    void init () {
        auto ticksPerMs = SystemCoreClock / 1000;
#if STM32G4
        if (SystemCoreClock > 150'000'000) // TODO use actual HPRE divider
            ticksPerMs /= 2; // HPRE is set to 2 (AHB freq must be <= 150 MHz)
#endif

        uint16_t next = cHead->mLen - ticks - 1;
        rate = next < 100 ? next+1 : 100;
//logf("r %d", rate);

        STK[0x4] = (rate * ticksPerMs) / 8 - 1; // reload value
        STK[0x0] = 0b011;                       // enable, clk/8 mode
    }

    void start (Message& msg) override {
        auto ms = msg.mLen;
        assert(ms <= 60'000);

        if (!isEmpty()) {
            auto next = (uint16_t) (cHead->mLen - ticks - 1);
            if (ms > next)
                ms = next;
        }

        if (ms < rate) {
//logf("s %d %d", ms, rate);
            STK[0x0] = 0;     // stop the clock
            ticks = millis(); // update actual tick count
        }

        auto t = millis();
        auto pp = &cHead; // insert in proper position
        while (*pp != nullptr && msg.mLen >= (uint16_t) ((*pp)->mLen - t))
            pp = &(*pp)->mLnk;

        msg.mLen += t; // make absolute, truncated to 16 bits
        msg.mLnk = *pp;
        *pp = &msg;

        if (STK[0x0] == 0)
            init();
    }

    void finish () override {
        while (expired())
            reply(pull());
        if (isEmpty()) {
//logf("e %d @ %d", rate, ticks);
            STK[0x0] = 0; // disable
        } else
            init();
    }

    bool interrupt (int) override {
        ticks += rate;
        assert(cHead != nullptr);
        uint16_t next = cHead->mLen - ticks - 1;
//logf("n %d r %d @ %d", next, rate, ticks);
        return next < rate || next > 60'000;
    }

    bool expired () const {
        return !isEmpty() && (uint16_t) (cHead->mLen - millis() - 1) > 60'000;
    }

    uint32_t millis () const {
        // the result has millisecond resolution, even when rate > 1
        while (true) {
            uint32_t t = ticks, n = STK[0x08];
            if (t == ticks)
                return t - (n*8)/(SystemCoreClock/1000);
        } // ticked just now, spin one more time
    }
};

} // namespace inline

extern "C"
void SysTick_Handler () {
    Device::byId('@').irqTrigger(0);
}

void sys::wait (uint16_t ms) {
    static Ticker ticker;
    Message m { ticker.dId, 'T', ms };
    call(m);
}

#endif // STM32
