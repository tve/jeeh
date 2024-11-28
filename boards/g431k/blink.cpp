#include <jee.h>

static void delayLoop (int n) {
    for (int i = 0; i < n * 5000; ++i)
        asm ("");
}

int main () {
    jeeh::Pin led ("B8","P");

    while (true) {
        led = 1;
        delayLoop(100);
        led = 0;
        delayLoop(900);
    }
}
