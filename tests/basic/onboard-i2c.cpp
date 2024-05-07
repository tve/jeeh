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
#if STM32L475xx // f475d
    printf("PB11 + PB10:\n");
    i2c.init("B11,B10");
    i2c.detect(); // 7 devices on I2C2
#endif
}
