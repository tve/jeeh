#include <jee.h>
using namespace jeeh;

void jeeh::fail (char const*, int) {
    while (true) {}
}

int main () {
    itmWrite("TEST\n"
             "OK\n", 8);
}
