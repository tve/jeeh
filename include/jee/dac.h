// Hardware digital to analog converter.

#if !STM32WL

namespace jeeh::dac {

enum { CR=0x00, DHR12R1=0x08, MCR=0x3C };

void init () {
    RCC(ena::DAC1,1) = 1;
    DAC1[MCR](14,2) = SystemCoreClock / 80'000'000; // 0, 1, or 2
    DAC1[CR](0) = 1; // EN1
}

void deinit () {
    RCC(ena::DAC1,1) = 0;
}

void set (uint32_t v) {
    DAC1[DHR12R1] = v;
}

} // namespace jeeh::dac

#endif
