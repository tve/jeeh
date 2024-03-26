#include <jee.h>
using namespace jeeh;

void jeeh::fail (char const*, int) {
    while (true) {}
}

void jeeh::failAt (void const*, char const* f, int n) {
    fail(f, n);
}

int main () {
    itmWrite("TEST\n"
             "OK\n", 8);
}
