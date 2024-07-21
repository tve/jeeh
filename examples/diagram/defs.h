//CG1 board leds
#define LED  "A1"

Pin led (LED);

namespace jeeh {
    Pin tracePins [15];
}

void initBoard () {
    fastClock();
    rtc::init(false);
    led.mode("P");

    Pin::config("A2:P,A7,A15,B7,A4,A1,A0,B3,A12,B0,B6,A8,A11,B5,B4",
                    tracePins, sizeof tracePins);
    trace(INIT);
}
