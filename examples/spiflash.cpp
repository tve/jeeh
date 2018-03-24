// boilerplate

#include <jee.h>

UartDev<Pin<'A',9>,Pin<'A',10>> uart;

void printf(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); veprintf(uart.putc, fmt, ap); va_end(ap);
}

// the spi flash chip is attached to the SPI1 pins

#include <jee/spi-flash.h>

SpiFlash<SpiDev<Pin<'A',7>,Pin<'A',6>,Pin<'A',5>,Pin<'A',4>>> mem;

int main () {
    printf("id %06x, %dK\n", mem.devId(), mem.size());

    mem.erase(0);
    printf("page 0 erased\n");

    uint8_t buf [256];
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 256; ++j)
            buf[j] = i + j;
        mem.write256(i, buf);
    }
    printf("10 pages written\n");

    for (int i = 0; i < 10; ++i) {
        printf("page %d:", i);
        mem.read256(i, buf);
        for (int j = 0; j < 15; ++j)
            printf(" %d", buf[j]);
        printf("\n");
    }

    mem.wipe();
    printf("wiped\n");

    mem.read256(0, buf);
    printf("buf[0] = %d\n", buf[0]);

    printf("done\n");
}

// sample output:
//
//  id C84017, 8192K
//  page 0 erased
//  10 pages written
//  page 0: 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14
//  page 1: 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
//  page 2: 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
//  page 3: 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17
//  page 4: 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18
//  page 5: 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19
//  page 6: 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
//  page 7: 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21
//  page 8: 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22
//  page 9: 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23
//  wiped
//  buf[0] = 255
//  done
