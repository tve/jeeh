// Floating point BMP390 sensor calculations.

#include <math.h>

uint8_t config [] = {
    0x1C, 0b00000000, // 0x1C OSR
    0x1D, 0b00000000, // 0x1D ODR
    0x1B, 0b00110011, // pwr_ctrl 0x1B
    0x18, 0b00001000, // FIFO_CONFIG2 0x18
    0x17, 0b00011001, // FIFO_CONFIG1 0x17
    //0x11, 0b00000011, // INT_STATUS 0x11
    //0x10, 0b00000011, // EVENT 0x10
    //0x00, 0x60,       // CHIP_ID
};

#if 0
enum {
    CHIP_ID       = 0x00, // Chip ID register sub-address
    ERR_REG       = 0x02, // Error register sub-address
    STATUS        = 0x03, // Status register sub-address
    DATA_0        = 0x04, // Pressure eXtended Least Significant Byte (XLSB) register sub-address
    DATA_1        = 0x05, // Pressure Least Significant Byte (LSB) register sub-address
    DATA_2        = 0x06, // Pressure Most Significant Byte (MSB) register sub-address
    DATA_3        = 0x07, // Temperature eXtended Least Significant Byte (XLSB) register sub-address
    DATA_4        = 0x08, // Temperature Least Significant Byte (LSB) register sub-address
    DATA_5        = 0x09, // Temperature Most Significant Byte (MSB) register sub-address
    SENSORTIME_0  = 0x0C, // Sensor time register 0 sub-address
    SENSORTIME_1  = 0x0D, // Sensor time register 1 sub-address
    SENSORTIME_2  = 0x0E, // Sensor time register 2 sub-address
    EVENT         = 0x10, // Event register sub-address
    INT_STATUS    = 0x11, // Interrupt Status register sub-address
    FIFO_LENGTH_0 = 0x12, // FIFO Length Least Significant Byte (LSB) register sub-address
    FIFO_LENGTH_1 = 0x13, // FIFO Length Most Significant Byte (MSB) register sub-address
    FIFO_DATA     = 0x14, // FIFO Data register sub-address
    FIFO_WTM_0    = 0x15, // FIFO Water Mark Least Significant Byte (LSB) register sub-address
    FIFO_WTM_1    = 0x16, // FIFO Water Mark Most Significant Byte (MSB) register sub-address
    FIFO_CONFIG_1 = 0x17, // FIFO Configuration 1 register sub-address
    FIFO_CONFIG_2 = 0x18, // FIFO Configuration 2 register sub-address
    INT_CTRL      = 0x19, // Interrupt Control register sub-address
    IF_CONFIG     = 0x1A, // Interface Configuration register sub-address
    PWR_CTRL      = 0x1B, // Power Control register sub-address
    OSR           = 0x1C, // Oversampling register sub-address
    ODR           = 0x1D, // Output Data Rate register sub-address
    CONFIG        = 0x1F, // Configuration register sub-address
    TRIM_PARAMS   = 0x31, // Trim parameter registers' base sub-address
    CMD           = 0x7E, // Command register sub-address
};
#endif

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
};          

FloatParams fp;

constexpr float pOf2 (uint8_t exp) { return powf(2, exp); }

void loadParams (TrimCoeffs& tc) {
    fp.T1 = tc.T1 * pOf2(8);
    fp.T2 = tc.T2 / pOf2(30);
    fp.T3 = tc.T3 / pOf2(48);
    fp.P1 = (tc.P1 - (1<<14)) / pOf2(20);
    fp.P2 = (tc.P2 - (1<<14)) / pOf2(29);
    fp.P3 = tc.P3 / pOf2(32);
    fp.P4 = tc.P4 / pOf2(37);
    fp.P5 = tc.P5 * pOf2(3);
    fp.P6 = tc.P6 / pOf2(6);
    fp.P7 = tc.P7 / pOf2(8);
    fp.P8 = tc.P8 / pOf2(15);
    fp.P9 = tc.P9 / pOf2(48);
    fp.P10 = tc.P10 / pOf2(48);
    fp.P11 = tc.P11 / pOf2(65);
}

float tComp (float ut) {
    auto pd1 = ut - fp.T1;
    auto pd2 = pd1 * fp.T2;
    return pd2 + pd1 * pd1 * fp.T3;  
}

float pComp (float up, float tl) {
    auto pd1 = fp.P6 * tl;
    auto pd2 = fp.P7 * tl * tl;
    auto pd3 = fp.P8 * tl * tl * tl;
    auto po1 = fp.P5 + pd1 + pd2 + pd3;
    pd1 = fp.P2 * tl;
    pd2 = fp.P3 * tl * tl;
    pd3 = fp.P4 * tl * tl * tl;
    auto po2 = up * (fp.P1 + pd1 + pd2 + pd3);
    pd1 = up * up;
    pd2 = fp.P9 + fp.P10 * tl;
    pd3 = pd1 * pd2;
    auto pd4 = pd3 + up * up * up * fp.P11;
    return po1 + po2 + pd4;
}

void showReading (uint8_t const* buf) {
    //logDump(buf, sizeof buf);

    auto t = (buf[5] << 16) | (buf[4] << 8) | buf[3];
    auto p = (buf[2] << 16) | (buf[1] << 8) | buf[0];

    auto ft = tComp(t);
    auto fp = pComp(p, ft);
    int32_t ct = ft * 1000;
    int32_t cp = fp * 10;

    logf("t-raw %d p-raw %d => temp %d.%03d °C, pres %d.%03d hPa",
            t, p, ct / 1000, ct % 1000, cp / 1000, cp % 1000);
}
