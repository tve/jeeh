// Hardware analog to digital converter.

namespace jeeh::adc {

enum { ISR=0x00, CR=0x08, CHSELR=0x30, DR=0x40 };

void init () {
    RCC(ena::ADC12,1) = 1;
    RCC[0x88](28,2) = 2; // CCIPR: ADC12SEL system clock
    ADC12_COMMON[0x08](16,2) = 3; // CCR: CKMODE /4

    ADC2[CR](29) = 0; // ~DEEPPWD
    ADC2[CR](28) = 1; // ADVREGEN
    cycles::usBusy(20);

    ADC2[CR](31) = 1; // ADCAL
    while (ADC2[CR](31)) {} // wait for calibration complete
    ADC2[ISR] = 1<<0; // ~ADRDY
    ADC2[CR](0) = 1; // ADEN
    while (!ADC2[ISR](0)) {} // ADRDY
}

void deinit () {
    RCC(ena::ADC12,1) = 0;
}

uint16_t read (uint8_t chan) {
    ADC2[CHSELR] = chan << 6;
    ADC2[CR](2) = 1; // ADSTART
//cycles::clear();
    while (ADC2[CR](2)) {} // wait until done
//logf("isr %08x %d", +ADC2[ISR], cycles::count());
    return ADC2[DR];
}

} // namespace jeeh::adc
