// Floating point BMP390 sensor calculations.

#pragma once
#include <cmath>

uint8_t config [] = {
    0x1C, 0x00, // OSR
    0x1D, 0x00, // ODR
    0x1B, 0x33, // PWR_CTRL
    0x18, 0x08, // FIFO_CONFIG2
    0x17, 0x19, // FIFO_CONFIG1
};

struct [[gnu::packed]] TrimCoeffs {                                                    
    uint16_t T1;
    uint16_t T2;
    int8_t   T3;
    int16_t  P1;
    int16_t  P2;
    int8_t   P3;
    int8_t   P4;
    uint16_t P5;
    uint16_t P6;
    int8_t   P7;
    int8_t   P8;
    int16_t  P9;
    int8_t   P10;
    int8_t   P11;
};
static_assert(sizeof (TrimCoeffs) == 21);
    
struct FloatParams {                                        
    float T1, T2, T3, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;

    void load (TrimCoeffs& tc) {
        // the ldexpf results are all computed at compile time
        T1 = tc.T1 * ldexpf(1, 8);
        T2 = tc.T2 / ldexpf(1, 30);
        T3 = tc.T3 / ldexpf(1, 48);
        P1 = (tc.P1 - (1<<14)) / ldexpf(1, 20);
        P2 = (tc.P2 - (1<<14)) / ldexpf(1, 29);
        P3 = tc.P3 / ldexpf(1, 32);
        P4 = tc.P4 / ldexpf(1, 37);
        P5 = tc.P5 * ldexpf(1, 3);
        P6 = tc.P6 / ldexpf(1, 6);
        P7 = tc.P7 / ldexpf(1, 8);
        P8 = tc.P8 / ldexpf(1, 15);
        P9 = tc.P9 / ldexpf(1, 48);
        P10 = tc.P10 / ldexpf(1, 48);
        P11 = tc.P11 / ldexpf(1, 65);
    }

    float tComp (float ut) const {
        auto pd1 = ut - T1;
        auto pd2 = pd1 * T2;
        return pd2 + pd1 * pd1 * T3;
    }

    float pComp (float up, float tl) const {
        auto pd1 = P6 * tl;
        auto pd2 = P7 * tl * tl;
        auto pd3 = P8 * tl * tl * tl;
        auto po1 = P5 + pd1 + pd2 + pd3;
        pd1 = P2 * tl;
        pd2 = P3 * tl * tl;
        pd3 = P4 * tl * tl * tl;
        auto po2 = up * (P1 + pd1 + pd2 + pd3);
        pd1 = up * up;
        pd2 = P9 + P10 * tl;
        pd3 = pd1 * pd2;
        auto pd4 = pd3 + up * up * up * P11;
        return po1 + po2 + pd4;
    }
};

FloatParams fp;

void showReading (uint8_t const* buf) {
    //logDump(buf, sizeof buf);

    auto t = (buf[5] << 16) | (buf[4] << 8) | buf[3];
    auto p = (buf[2] << 16) | (buf[1] << 8) | buf[0];

    auto t2 = fp.tComp(t);
    auto p2 = fp.pComp(p, t2);
    int32_t ct = t2 * 1000;
    int32_t cp = p2 * 10;

    logf("t-raw %08x p-raw %08x  =>  temp %d.%03d °C, pres %d.%03d hPa",
            t, p, ct / 1000, ct % 1000, cp / 1000, cp % 1000);
}
