// interface definitions for hardware I/O register access, based on SVD

template< uint32_t A >
struct IoReg {
    static constexpr auto ADDR = A;
#if STM32F1 | STM32F3 | STM32F4 | STM32L4
    static constexpr auto CAN_BIT_BAND = (A>>20) == 0x200 || (A>>20) == 0x400;
#else
    static constexpr auto CAN_BIT_BAND = false;
#endif

    struct Bits {
        uint32_t o;
        uint8_t b, w;

        operator int () const {
            if (CAN_BIT_BAND && w == 1)
                return *bitBandAddr();
            auto mask = (1<<w)-1;
            auto ptr = (volatile uint32_t*) (A+o);
            return (*ptr>>b) & mask;
        }

        [[gnu::always_inline]]
        int operator= (int v) const {
            if (CAN_BIT_BAND && w == 1)
                return *bitBandAddr() = v;
            auto mask = (1<<w)-1;
            auto ptr = (volatile uint32_t*) (A+o);
            *ptr = (*ptr & ~(mask<<b)) | ((v&mask)<<b);
            return v;
        }

        int operator= (Bits const& v) const {
            return operator= ((int) v);
        }

        [[gnu::always_inline]]
        constexpr auto bitBandAddr () const {
            return (volatile uint32_t*) ((A&0xF000'0000) + 0x0200'0000 +
                                                    ((A+o)<<5) + (b<<2));
        }
    };

    struct Word {
        uint32_t o;

        constexpr auto operator() (uint8_t bit, uint8_t width =1) const {
            return Bits{ o+4*(bit/32), (uint8_t) (bit%32), width };
        }

        operator int () const {
            return *(volatile uint32_t*) (A+o);
        }

        int operator= (int v) const {
            *(volatile uint32_t*) (A+o) = v;
            return v;
        }

        int operator= (Word const& v) const {
            return operator= ((int) v);
        }
    };

    constexpr auto operator[] (uint32_t off) const {
        return Word{ off }; // this is a *byte* offset
    }

    constexpr auto operator() (uint32_t bit, uint8_t width) const {
        return operator[](4*(bit/32))(bit%32, width);
    }

    constexpr auto& byte (uint32_t off) const {
        return *(volatile uint8_t*) (A+off);
    }

    constexpr auto& half (uint32_t off) const {
        return *(volatile uint16_t*) (A+off); // this is a *byte* offset
    }
};
