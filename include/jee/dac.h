// Hardware digital to analog converter.

namespace jeeh::dac {

enum { CR=0x00, DHR12R1=0x08 };

void init () {
    RCC(ena::DAC1,1) = 1;
    DAC1[CR](0) = 1; // EN1
}

void deinit () {
    RCC(ena::DAC1,1) = 0;
}

void set (uint32_t v) {
    DAC1[DHR12R1] = v;
}

} // namespace jeeh::dac
