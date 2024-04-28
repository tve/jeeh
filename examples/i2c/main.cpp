// This code can be compiled with either "arduino" or "cmsis" as PIO framework.
// I.e. "pio run -e l432k-arduino" or "pio run -e l432-cmsis".

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

enum { HMC5883=0x1E, ADXL345=0x53, ITG3200=0x68 };

constexpr Pin led ("B3"); // Nucleo-L432KC

void jeeh::fail(void const* a, char const* f, int n) {
    printf("\nfailed at %s:%d\nfailed caller: %p\n", f, n, a);
    while (true) {}
}

void jeeh::hardFaultHandler (uint32_t* sp) {
    printf("\nhard fault, sp = %p", sp);
    fail();
}

Uart uart ('U'); // DMA/IRQ-based UART device driver, id = 'U'

extern "C" int _write (int, char* ptr, int len) {
    Message m { 'U', 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

int main () {
    hardFaulter = hardFaultHandler;
    fastClock();
    cycles::init();

    // the UART config comes from platformio.ini and is defined in defs.h
    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });

    printf("???\r%s @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);
    led.mode("P"); // push-pull output

    I2cGpio i2c;

    printf("SDA=PB5, SCL=PB4\n");
    i2c.init("B5,B4");
    i2c.detect(); // look for audio codec

    uint8_t buf [6];

    for (auto i = 0; i < 3; ++i) {
        sys::wait(50);
        i2c.readRegs(HMC5883, 3, buf, sizeof buf);

        auto x = (int16_t) ((buf[0] << 8) | buf[1]);
        auto y = (int16_t) ((buf[2] << 8) | buf[3]);
        auto z = (int16_t) ((buf[4] << 8) | buf[5]);
        printf("HMC5883 compass: xyz = %d %d %d\n", x, y, z);
    }

    i2c.writeReg(ADXL345, 0x2D, 0x08); sys::wait(10); // Measurement mode
    i2c.writeReg(ADXL345, 0x31, 0x08); sys::wait(10); // full resolution
    i2c.writeReg(ADXL345, 0x2C, 0x09); sys::wait(10); // 50 Hz rate

    for (auto i = 0; i < 3; ++i) {
        sys::wait(50);
        i2c.readRegs(ADXL345, 0x32, buf, sizeof buf);

        auto x = ((int8_t) buf[1] << 8) | buf[0];
        auto y = ((int8_t) buf[3] << 8) | buf[2];
        auto z = ((int8_t) buf[5] << 8) | buf[4];
        printf("ADXL345 accel:   xyz = %d %d %d\n", x, y, z);
    }

    i2c.writeReg(ITG3200, 0x3E, 0x80); sys::wait(10); // full-scale range, 42 Hz
    i2c.writeReg(ITG3200, 0x16, 0x1B); sys::wait(10); // DLPF_CFG=3, FS_SEL=3
    i2c.writeReg(ITG3200, 0x15, 0x0A); sys::wait(10); // SMPLRT_DIV = 10 (50Hz)
    i2c.writeReg(ITG3200, 0x3E, 0x03); sys::wait(10); // PLL with z gyro ref

    for (auto i = 0; i < 3; ++i) {
        sys::wait(50);
        i2c.readRegs(ITG3200, 0x1D, buf, sizeof buf);

        auto x = (int16_t) ((buf[0] << 8) | buf[1]);
        auto y = (int16_t) ((buf[2] << 8) | buf[3]);
        auto z = (int16_t) ((buf[4] << 8) | buf[5]);
        printf("ITG3200 gyro:    xyz = %d %d %d\n", x, y, z);
    }

    while (true) {
        sys::wait(250);
        led.toggle();
    }
}
