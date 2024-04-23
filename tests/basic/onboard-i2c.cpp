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

#if STM32F723xx
    printf("PB9 + PB8:\n");
    i2c.init("B9,B8");
    i2c.detect(); // look for audio codec

    printf("PH8 + PA8:\n");
    i2c.init("H8,A8");
    i2c.detect(); // look for touch panel
#endif
}
