
// busy-wait for a condition with a timeout and printing a value on timeout
#define WHILE(cond, ms, value) \
    for(uint32_t t0=cycles::millis(), to=(ms); (cond); ) { \
        if (cycles::millis() > t0+to) { \
            uint32_t v = (value); \
            printf("WAIT timed out: cond=<<%s>>, val=%ld=0x%lx\n", #cond, v, v); \
            t0 = cycles::millis(); to = 2000; \
        } \
    }

namespace jeeh {

struct Analog {
    // registers
    enum { ISR=0x00, CR=0x08, CFGR1=0x0C, CFGR2=0x10, CHSEL=0x28, DR=0x40, CALFACT=0xb4, CCR=0x308 };
    auto& adcReg (int off) const { return regs[off/4]; }
    

    // internal channels
#if STM32WL
    enum { Vtemp=12, Vrefint, Vbat3, Vdac1=17 };
    constexpr static uint32_t vrefint_vdd = 3300; // vrefint_cal is @VDD=3300mV
    constexpr static uint32_t vrefint_cal = 0x1fff75AA; // ADC Vref @3V
    constexpr static uint32_t temp30_cal  = 0x1fff75A8; // ADC @30C
    constexpr static uint32_t temp130_cal = 0x1fff75C8; // ADC @130C
#elif STM32L0
    enum { Vrefint=17, Vtemp };
    constexpr static uint32_t vrefint_vdd = 3000; // vrefint_cal is @VDD=3000mV
    constexpr static uint32_t vrefint_cal = 0x1ff80078; // ADC Vref @3V
    constexpr static uint32_t temp30_cal = 0x1ff8007A; // ADC @30C
    constexpr static uint32_t temp130_cal = 0x1ff8007E; // ADC @130C
#elif STM32L4
    // oops, way more complicated, has two ADCs plus an extra channel selection mux stage
    // enum { Vrefint=0, Vrefneg=0, Vdac1=17, Vdac2 }; // ADC1
    // enum {VRef}
#endif

    struct Config {
        uint32_t addr; // ADC.ADDR
        uint16_t ena; // ena::ADC
        bool adcClk; // true:adcClk(e.g. hsi16), false:PCLK/2 clock (to avoid 50% duty cycle issue)
        uint8_t oversample; // oversampling in HW, 0:none, 1:2x, 2:4x, ... 8:256x
    };

    Config const dev;
    volatile uint32_t *regs;
    uint8_t calfact = 0x80; // calibration factor (7-bit value)

    Analog (Config const c) : dev(c), regs ((volatile uint32_t*) c.addr) {}

    // initialize and power the ADC up
    void init () {
        RCC(dev.ena, 1) = 1;

        // clock source (should some of the values be precomputed in constructor?)
        if (adcReg(CR) & 1) { // ADC is enabled (should not happen), need to disable to configure
            adcReg(CR) = 1<<1; // set ADDIS: ensure ADC is disabled
            while (adcReg(CR) & 3) ; // wait for disable to take effect (important!)
        }
        uint32_t cfgr2 = 0;
        uint8_t os = dev.oversample; // keeps the next line short...
        if (os) cfgr2 = (0<<9) | (os<<5) | ((os-1)<<2) | (1<<0); // ~TOVS OVSS(shift) OVSR(ratio)
        adcReg(CFGR2) = (((!dev.adcClk)&1)<<30) | cfgr2;
        RCC(28 + 8*0x88, 2) = 3; // select sysclk as ADC clock source

        adcReg(CCR) = 0; // (1<<24) | (1<<23) | (1<<22); // VBATEN TSEN VREFEN
        adcReg(CFGR1) = 0; // ensure known state during init/deinit/init sequences

#if 0
        adcReg[ccr] |= 1<<25; // set low-freq mode (ADC clock freq < 3.5Mhz)
#endif

#if STM32WL
        // enable ADC voltage regulator
        adcReg(CR) |= 1<<28; // ADVREGEN
        cycles::usBusy(20); // need to wait Tadcvreg_stup: 20us for stm32wle5
#endif

        // calibration
        if (calfact == 0x80) {
            // perform calibration 4x and average
            calfact = 0;
            for (auto i=0; i<4; i++) {
                adcReg(CR) |= (1<<31); // set ADCAL -- start calibration
                while (adcReg(CR) & (1<<31)) ;  // wait until calibration completed
                calfact += adcReg(DR);
            }
            calfact = (calfact+2) / 4;
        }
        // set calibration value
        adcReg(CALFACT) = calfact;

        // finally enable ADC... ???
        adcReg(CR) = (1<<0); // set ADEN -- enable ADC
    }

    // power down the ADC
    void deinit () {
        uint32_t cr = adcReg(CR);
        if (cr & 1) { // ADC is enabled, need to go through disable process
            if (cr & (1<<2)) { // ADSTART == 1 -> conversion in progress
                adcReg(CR) = cr | (1<<4); // ADSTP -- stop conversion
                while (adcReg(CR) & (1<<4)) ; // wait for stop
            }
            adcReg(CR) |= 1<<1; // set ADDIS
            while (adcReg(CR) & 3) ; // wait for disable
        }
        adcReg(CR) = 0; // disable Vreg
        RCC(dev.ena, 1) = 0;
    }

    uint8_t pin2chan(Pin &pin) {
#if STM32WL
        constexpr int8_t map[5] = { -1, 5, 4, 2, 3 }; // N/A, PB1, PB2, PB3, PB4
        int ch = pin.port() == 0 ? pin.pin()-4 // PA10 -> IN6 .. PA15 -> IN11
               : pin.port() == 1 && pin.pin() > 13 ? pin.pin()-13 // PB13 -> IN0 .. PB14 -> IN1
               : pin.port() == 1 && pin.pin() < 5 ? map[pin.pin()]
               : -1;
        return ch;
#else
        constexpr int off = pin::id < 16 ? 0 :   // A0..A7 => 0..7
                            pin::id < 32 ? -8 :  // B0..B1 => 8..9
                                           -22;  // C0..C5 => 10..15
        return read(pin::id + off);
#endif
    }

    // read analog, given a pin (which is assumed to be already set to analog input mode)
    uint16_t read (Pin &pin) {
        return read(pin2chan(pin));
    }

    inline void selectChannel(uint8_t chan) {
        adcReg(CHSEL) = 1<<chan;
#if STM32WL
        WHILE ((adcReg(ISR) & (1<<13)) == 0, 10, adcReg(ISR)); // wait until CCRDY
#endif
    }

    // read direct channel number
    uint16_t read (uint8_t chan) {
        uint32_t cr = adcReg(CR);
        assert(cr & 1); // ADC must already be enabled
        adcReg(ISR) = 0xff; // clear all flags, needed for CCRDY at least
        selectChannel(chan);
        adcReg(CR) = cr | (1<<2);  // set ADSTART start conversion
        //printf("chan=%d sel=%x cr=%x\r\n", chan, adcReg(chsel), adcReg(cr));
        //printf("cr=%x isr=%x\r\n", adcReg(cr), adcReg(isr));
        WHILE ((adcReg(ISR) & (1<<2)) == 0, 10, adcReg(ISR)) ;  // EOC (appropriate for HW oversampling too)
        return adcReg(DR);
    }

    // read current Vdd in millivolts
    uint16_t readVdd() {
        adcReg(CCR) |= 1<<22; // set VREF_EN bit
        uint32_t adc = read(Vrefint);
        adcReg(CCR) &= ~(1<<22); // clear VREF_EN bit
        uint32_t cal = *(uint16_t*)vrefint_cal;
        return vrefint_vdd * cal / adc;
    }

    // read current temperature in centigrade
    int16_t readTemp() {
        adcReg(CCR) |= 1<<23; // set TSEN bit now so temp sensor is ready by the time we read it
        int32_t vcc = readVdd();
        int32_t adc = read(Vtemp);
        adcReg(CCR) &= ~(1<<23); // clear TSEN bit
        int32_t cal30 = *(uint16_t*)temp30_cal;
        int32_t cal130 = *(uint16_t*)temp130_cal;
        int32_t temp = adc * vcc / vrefint_vdd - cal30;
        // printf("Temp: adc=%ld temp=%ld cal30=%ld cal130=%ld\n", adc, temp, cal30, cal130);
        return (int16_t)(temp * 100 / (cal130-cal30) + 30);
  }

};


}
