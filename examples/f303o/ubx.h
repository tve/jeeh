namespace ubx {

struct NavPvt {
    uint32_t iTOW;       // ms    GPS time of week of the navigation epoch
    uint16_t year;       // y     Year (UTC)
    uint8_t  month;      // month Month, range 1..12 (UTC)
    uint8_t  day;        // d     Day of month, range 1..31 (UTC)
    uint8_t  hour;       // h     Hour of day, range 0..23 (UTC)
    uint8_t  min;        // min   Minute of hour, range 0..59 (UTC)
    uint8_t  sec;        // s     Seconds of minute, range 0..60 (UTC)
    int8_t   valid;      //       Validity Flags (see graphic below)
    uint32_t tAcc;       // ns    Time accuracy estimate (UTC)
    int32_t  nano;       // ns    Fraction of second, range -1e9 .. 1e9 (UTC)
    uint8_t  fixType;    //       GNSSfix Type, range 0..5
                         //       0x00 = No Fix
                         //       0x01 = Dead Reckoning only
                         //       0x02 = 2D-Fix
                         //       0x03 = 3D-Fix
                         //       0x04 = GNSS + dead reckoning combined
                         //       0x05 = Time only fix
                         //       0x06..0xff: reserved
    int8_t   flags;      //       Fix Status Flags (see graphic below)
    uint8_t  reserved1;  //       Reserved
    uint8_t  numSV;      //       Number of satellites used in Nav Solution
    int32_t  lon;        // deg   Longitude (1e-7)
    int32_t  lat;        // deg   Latitude (1e-7)
    int32_t  height;     // mm    Height above Ellipsoid
    int32_t  hMSL;       // mm    Height above mean sea level
    uint32_t hAcc;       // mm    Horizontal Accuracy Estimate
    uint32_t vAcc;       // mm    Vertical Accuracy Estimate
    int32_t  velN;       // mm/s  NED north velocity
    int32_t  velE;       // mm/s  NED east velocity
    int32_t  velD;       // mm/s  NED down velocity
    int32_t  gSpeed;     // mm/s  Ground Speed (2-D)
    int32_t  heading;    // deg   Heading of motion 2-D (1e-5)
    uint32_t sAcc;       // mm/s  Speed Accuracy Estimate
    uint32_t headingAcc; // deg   Heading Accuracy Estimate (1e-5)
    uint16_t pDOP;       //       Position DOP (0.01)
    int16_t  reserved2;  //       Reserved
    uint32_t reserved3;  //       Reserved
    int32_t  headVeh;    // deg   Heading of vehicle
    int16_t  magDec;     // deg   Magnetic declination
    uint16_t magAcc;     // deg   Magnetic declination accuracy
};
static_assert(sizeof (NavPvt) == 92);

template< uint16_t MAX >
struct Parser {
    enum { SYNC1, SYNC2, CLASS, MSGID, LEN1, LEN2, PAYLOAD, CRC1, CRC2 };

    uint16_t pktLen, pktFill;
    uint8_t state =SYNC1, pktClass, pktMsgId, pktCkA, pktCkB;
    uint8_t payload [MAX] alignas(4);

    bool parse (uint8_t ch) {
        if (CLASS <= state && state < CRC1) {
            pktCkA += ch;
            pktCkB += pktCkA;
        }
        switch (state) {
            case SYNC1:
                if (ch == 0xB5)
                    ++state;
                break;
            case SYNC2:
                if (ch == 0x62)
                    ++state;
                else
                    state = SYNC1;
                pktFill = pktCkA = pktCkB = 0;
                break;
            case CLASS: pktClass = ch; ++state; break;
            case MSGID: pktMsgId = ch; ++state; break;
            case LEN1:  pktLen = ch; ++state; break;
            case LEN2:
                pktLen |= ch<<8;
                ++state;
                if (pktLen == 0)
                    ++state; // empty payload
                break;
            case PAYLOAD:
                if (pktFill < sizeof payload)
                    payload[pktFill++] = ch;
                if (pktFill >= pktLen)
                    ++state;
                break;
            case CRC1:
                if (pktCkA != ch)
                    state = SYNC1;
                else
                    ++state;
                break;
            case CRC2:
                state = SYNC1;
                return pktCkB == ch;
        }
        return false;
    }
};

// see https://en.wikipedia.org/wiki/Maidenhead_Locator_System
char* maidenhead (char* buf, int32_t lat, int32_t lon) {
    auto fill = 0;
    constexpr uint32_t e7 = 10'000'000;
    uint32_t scale = 180*e7;
    uint32_t ulon = lon + 180*e7,  ulat = lat + 90*e7; // e-7
    for (auto i = 0; i < 4; ++i) {
        auto radix = i == 0 ? 18 : i & 1 ? 10 : 24;
        auto base = radix == 10 ? '0' : 'A';
        scale /= radix;
        buf[fill++] = base + (ulon / (2*scale)) % radix;
        buf[fill++] = base + (ulat / scale) % radix;
    }
    buf[fill] = 0;
    return buf;
}

} // namespace ubx
