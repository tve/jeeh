# I2C sensor readout on a Nucleo-32 L432KC

This expects a "9DOF Stick" attached to D4+D5 (i.e. SDA+SCL, GPIO PB7+PB6).

Sample output for `pio run -e detect -t upload`:

    STM32L4x2: detect @ 80 MHz
    00:                         -- -- -- -- -- -- -- --
    10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- 1E --
    20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    30: -- -- -- -- -- -- -- -- -- -- -- -- 3C 3D -- --
    40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    50: -- -- -- 53 -- -- -- -- -- -- -- -- -- -- -- --
    60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
    70: -- -- -- -- -- -- -- --
    oled: 4678 µs

Sample output for `pio run -e imu -t upload`:

    STM32L4x2: imu @ 80 MHz
    HMC5883 compass: xyz = -728   85  179
    HMC5883 compass: xyz = -726   83  178
    HMC5883 compass: xyz = -730   85  180
    ADXL345 accel:   xyz =   -3  -43  237
    ADXL345 accel:   xyz =   -2  -44  238
    ADXL345 accel:   xyz =    1  -39  243
    ITG3200 gyro:    xyz =   -6   29    5
    ITG3200 gyro:    xyz =   -6   29    4
    ITG3200 gyro:    xyz =   -7   30    5

Show a pattern on 2nd 128x64 OLED display with: `pio run -e oled -t upload`
