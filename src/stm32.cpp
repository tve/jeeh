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

//------------------------------------------------------------------------ ITM

#if !STM32G0 && !STM32L0 // Cortex M0+ doesn't support ITM

void itmWrite (void const* ptr, size_t len) {
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

struct Ticker : Device, Chain {

    Ticker () : Device ('@') {
        rate = 1; // TODO
        ticks += rate;
        auto ticksPerMs = SystemCoreClock / 1000;
#if STM32G4 && F_CPU > 150'000'000
        if (SystemCoreClock > 150'000'000)
            ticksPerMs /= 2; // HPRE is set to 2, since max AHB freq is 150 MHz
#endif
        STK[0x4] = (rate*ticksPerMs)/8-1; // reload value
        STK[0x8] = 0;                     // current
        STK[0x0] = 0b011;                 // control, clk/8 mode
    }

    void start (Message& msg) override {
logf("20");
        assert(msg.mLen <= 60'000);
        auto t = millis();

        auto pp = &cHead; // insert in proper position
        while (*pp != nullptr && msg.mLen >= (uint16_t) ((*pp)->mLen - t))
            pp = &(*pp)->mLnk;

        msg.mLen += t; // make absolute, truncated to 16 bits
        msg.mLnk = *pp;
        *pp = &msg;
logf("21");
    }

    void finish () override {
logf("22");
        while (expired())
            reply(pull());
logf("23");
    }

    bool interrupt (int) override {
logf("30");
        ticks += rate;
        return expired();
    }

    static uint32_t millis () {
        // the result has millisecond resolution, even when rate > 1
        while (true) {
            uint32_t t = ticks, n = STK[0x08];
            if (t == ticks)
                return t - (n*8)/(SystemCoreClock/1000);
        } // ticked just now, spin one more time
    }

    bool expired () const {
        return !isEmpty() && (uint16_t) (cHead->mLen - millis() - 1) > 60'000;
    }

    inline static volatile uint32_t ticks;
    inline static uint8_t rate;
};

Ticker ticker;

} // namespace jeeh

extern "C"
void SysTick_Handler () {
    jeeh::logf("tick!");
    jeeh::Device::byId('@').irqTrigger(0);
}

#endif // STM32
