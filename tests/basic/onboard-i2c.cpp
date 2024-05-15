#include <jee.h>
#include <jee/i2c.h>
using namespace jeeh;
#include "test.h"

extern "C" int _write (int, char* ptr, int len) {
    itmWrite(ptr, len);
    return len;
}

int main () {
    Tester t;

    I2cGpio i2c;

#if STM32F723xx // f723d
    printf("PB9 + PB8:\n");
    i2c.init("B9,B8");
    i2c.detect(); // audio codec

    printf("PH8 + PA8:\n");
    i2c.init("H8,A8");
    i2c.detect(); // touch panel
#endif
#if STM32L475xx // l475d
    printf("PB11 + PB10:\n");
    i2c.init("B11,B10");
    i2c.detect(); // 7 devices on I2C2
#endif
#if STM32L496xx // l496d
    Pin lcd ("H0"); lcd.mode("P"); lcd = 0;
    Pin mfx ("H6"); mfx.mode("P"); mfx = 1;
    Pin bkl ("I0"); bkl.mode("P"); bkl = 1;
    printf("PB14 + PH4:\n");
    i2c.init("B14,H4", 40);
    i2c.detect(); // 2 devices on I2C2
#endif
}
