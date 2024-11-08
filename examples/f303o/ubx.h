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

} // namespace ubx
