//CG1 board leds
#define LED  "A1"

Pin led (LED);
Pin pins [16];

void initBoard () {
    fastClock();
    led.mode("P");
    led = 1; // inverted logic

    AFIO[0x04](24,3) = 2; // disable non-SWD JTAG pins

    Pin::config("B10:P,B11,B12,B13,B14,B15,A8,A11,A12,A15,B3,B4,B5,B6,B7,B8",
                    pins, sizeof pins);
}
