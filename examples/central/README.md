**This is a "central node" setup based on the F723IE Discovery board.**

There's an RFM69 attached to the 20-pin P1-STMod+ connector:

GPIO | RFM69 | STMod+ | MBUS | Note
-----|-------|--------|------|-----
P    | DIO0  | 13     | 1L   | ADC
PF11 | NRST  | 12     | 2L   | RST
PI0  | NSS   | 1      | 3L   | NSS2
PI1  | SCK   | 4      | 4L   | SCK2
PI2  | MISO  | 9      | 5L   | MISO2
PI3  | MOSI  | 8      | 6L   | MOSI2
-    | +3.3  | -      | 7L   | 3.3V
-    | GND   | -      | 8L   | GND
PB0  | DIO1  | 14     | 1R   | PWM
PB11 | DIO2  | 11     | 2R   | INT
PH4  | DIO3  | 7      | 5R   | SCL
PH5  | DIO5  | 10     | 6R   | SDA

This assumes the PMOD selection pins are set as: PH15=1 and PI10=0.  
TX+RX are available for the 12-pin PMod, 8-pin ESP-01, or 4-pin Grove connector.
