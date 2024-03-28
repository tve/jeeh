#include <jee.h>
using namespace jeeh;

void jeeh::fail (void const*, char const*, int) {
    while (true) {}
}

int main () {
    itmWrite("TEST\n"
             "OK\n", 8);
}
