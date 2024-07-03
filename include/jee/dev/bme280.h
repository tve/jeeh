// Sensor calculations for the BME280 sensor.

template< typename DEV >
struct BME280 {
    DEV dev;

    BME280 (DEV const& d) : dev {d} {}

    struct [[gnu::packed]] TrimCoeffs {                                                    
        uint16_t T1;
        int16_t T2, T3;
        uint16_t P1;
        int16_t P2, P3, P4, P5, P6, P7, P8, P9;
        uint8_t H1;
        int16_t H2;
        uint8_t H3;
        int16_t H4, H5;
        int8_t H6;

        // Return temperature in 0.01 C.
        int32_t tComp(int32_t tRaw, int32_t& tFine) const {
            auto v1 = ((((tRaw>>3) - (T1<<1))) * (T2)) >> 11;
            auto v2 = (((((tRaw>>4) - T1) * ((tRaw>>4) - T1)) >> 12) * T3) >> 14;
            tFine = v1 + v2;
            return (tFine * 5 + 128) >> 8;
        }

        // Return pressure in Pa using Q24.8 format.
        uint32_t pComp(int32_t pRaw, int32_t tFine) const {
            auto v1 = tFine - 128000LL;
            auto v2 = v1 * v1 * P6;
            v2 = v2 + ((v1*P5) << 17);
            v2 = v2 + (((int64_t)P4) << 35);
            v1 = ((v1 * v1 * P3) >> 8) + ((v1 * P2) << 12);
            v1 = (((1LL<<47) + v1)) * P1 >> 33;
            if (v1 == 0)
                return 0; // avoid exception caused by division by zero
            int64_t p = 1048576 - pRaw;
            p = (((p<<31) - v2) * 3125) / v1;
            v1 = (((int64_t)P9) * (p>>13) * (p>>13)) >> 25;
            v2 = (P8 * p) >> 19;
            return ((p + v1 + v2) >> 8) + (P7<<4);
        }

        // Return humidity in %RH using Q22.10 format.
        uint32_t hComp(int32_t hRaw, int32_t tFine) const {
            int32_t v = tFine - 76800;
            v = (((hRaw << 14) - (H4 << 20) - (H5 * v) + (1<<14)) >> 15) *
                (((((((v * H6) >> 10) * (((v * H3) >> 11) + (1<<15))) >> 10) +
                (1<<21)) * H2 + (1<<13)) >> 14);
            v -= ((((v >> 15) * (v >> 15)) >> 7) * H1) >> 4;
            v = v < 0 ? 0 : v > (100<<22) ? (100<<22) : v;
            return v>>12;
        }

        // Return compensated results from 8-byte input buffer.
        auto convert (uint8_t const* buf) const {
            auto t = (buf[3]<<12) | (buf[4]<<4) | (buf[5]>>4);
            auto p = (buf[0]<<12) | (buf[1]<<4) | (buf[2]>>4);
            auto h = (buf[6]<<8) | buf[7];
            //logf("t-raw %d p-raw %d h-raw %d", t, p, h);

            int32_t tFine;
            auto ct = tComp(t, tFine);
            auto cp = pComp(p, tFine);
            auto ch = hComp(h, tFine);

            cp = (cp * 100 + 128) >> 8;   // Q24.8 -> x100
            ch = (ch * 1000 + 512) >> 10; // Q22.10 -> x1000

            struct Result { int32_t t; uint32_t p, h; };
            return Result{ ct, cp, ch };
        }
    };
    static_assert(sizeof (TrimCoeffs) == 33);

    TrimCoeffs tc;

    void init () {
        dev.write(0xF2, 1);
        dev.write(0xF4, (1<<5) | (1<<2) | 3);
        dev.write(0xF5, (3<<5) | (0<<2) | 0);

        dev.read(0x88, &tc.T1, 24);
        dev.read(0xA1, &tc.H1, 1);
        dev.read(0xE1, &tc.H2, 7);
        // unpack last few params
        tc.H6 = tc.H5 >> 8;
        tc.H5 = ((int8_t) tc.H5 << 4) | ((tc.H4 >> 12) & 0x0F);
        tc.H4 = ((int8_t) tc.H4 << 4) | ((tc.H4 >> 8) & 0x0F);
    }

    auto getReading () const {
        uint8_t buf [8];
        dev.read(0xF7, buf, sizeof buf);
        return tc.convert(buf);
    }
};
