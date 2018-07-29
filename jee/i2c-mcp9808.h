// Driver for a MCP9808 temperature sensor

template< typename I2C, int addr =0x18 >
struct MCP9808 {

    static bool init () {
        for (int i=1; i<8; i++)
            printf("reg %d = 0x%x\r\n", i, readReg(i));
        if (readReg(6) != 0x54 || readReg(7) != 0x400) return false;
        writeReg(1, 0x100); // set shutdown mode
        writeReg(8, 2);     // set resolution 2=0.125C, 3=0.0625C
        return true;
    }

    // convert puts the device into continuous conversion mode and return the number of
    // milliseconds before the first conversion completes.
    static uint32_t convert() {
        writeReg(1, 0); // set continuous conversion mode
        return 130;
    }

    // read returns the current temperature in 1/100th centigrade, assume a conversion is ready
    static int32_t read() {
        int32_t v = readReg(5);
        v = (v<<19)>>19; // sign-extend
        return (v*100 + 8) / 16;
    }

    static void sleep() {
        writeReg(1, 0x100);
    }

    static uint16_t readReg (uint8_t r) {
        I2C::start(addr<<1);
        I2C::write(r);
        I2C::stop();
        I2C::start((addr<<1)|1);
        uint16_t v = I2C::read(false);
        return (v<<8) | I2C::read(true);
    }

    static void writeReg (uint8_t r, uint16_t v) {
        I2C::start(addr<<1);
        I2C::write(r);
        if (r != 8) I2C::write(v>>8);
        I2C::write(v&0xff);
        I2C::stop();
    }
};

