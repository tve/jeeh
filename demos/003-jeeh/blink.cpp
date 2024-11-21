#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;

Pin led {"C13","P"};

int main () {
    cycles::init();
	
    while (true) {
        led = 0;
        cycles::msBusy(100);
        led = 1;
        cycles::msBusy(400);
    }
}
