#include <jee.h>

UartDev<Pin<'A',9>,Pin<'A',10>> uart;

void printf(const char* fmt, ...) {
    va_list ap;
    va_start (ap, fmt);
    veprintf(uart.putc, fmt, ap);
    va_end (ap);
}

int main () {
    printf(" d %d o %o x %x b %b c %c",
            '!', '!', '!', '!', '!');
    printf(" %%3d <%3d> %%03d <%03d> %%s <%s> %%3s <%3s> ",
            -1, 12, "ab", "ab");
    uart.puts("Hello!\n");
}

// output:
//  d 33 o 41 x 21 b 100001 c ! %3d < -1> %03d <012> %s <ab> %3s <ab > Hello!
