**Nucleo-32 STM32 G431 or L432 with GPIO pins tied to a Logic Analyser.**

    Conn | Name  | LA | G431 | Notes | L432 | Notes 
    -----|-------|----|------|-------|------|-------
    R-1  | VIN   |    | -    |       | -    |       
    R-2  | GND   |    | -    |       | -    |       
    R-3  | TNRST | 2  | RST  |       | RST  |       
    R-4  | 5V    |    | -    |       | -    |       
    R-5  | A7    | 4  | PA2  | UART  | PA2  | UART  
    R-6  | A6    | 5  | PA7  |       | PA7  |       
    R-7  | A5    | 6  | PA15 | I2C   | PA6  |       
    R-8  | A4    | 7  | PB7  | I2C   | PA5  |       
    R-9  | A3    | 8  | PA4  |       | PA4  |       
    R-10 | A2    |    | PA3  | UART  | PA3  |       
    R-11 | A1    | 9  | PA1  |       | PA1  |       
    R-12 | A0    | 10 | PA0  |       | PA0  |       
    R-13 | AVDD  |    | -    |       | -    |       
    R-14 | 3V3   |    | -    |       | -    |       
    R-15 | D13   | 13 | PB3  | SPI   | PB3  | SPI   
    L-1  | D1    |    | PA9  | UART  | PA9  | UART  
    L-2  | D0    |    | PA10 | UART  | PA10 | UART  
    L-3  | TNRST |    | RST  | dupl  | RST  | dupl  
    L-4  | GND   |    | -    |       | -    |       
    L-5  | D2    | 0  | PA12 |       | PA12 |       
    L-6  | D3    | 1  | PB0  |       | PB0  |       
    L-7  | D4    |    | PB7  | dupl  | PB7  |       
    L-8  | D5    |    | PA15 | dupl  | PB6  |       
    L-9  | D6    | 3  | PB6  |       | PB1  |       
    L-10 | D7    |    | PF0  | OSC   | PC14 | OSC   
    L-11 | D8    |    | PF1  | OSC   | PC15 | OSC   
    L-12 | D9    | 11 | PA8  |       | PA8  |       
    L-13 | D10   | 12 | PA11 | SPI   | PA11 | SPI   
    L-14 | D11   | 14 | PB5  | SPI   | PB5  | SPI   
    L-15 | D12   | 15 | PB4  | SPI   | PB4  | SPI   

**Logic Analyser hookup:**

![](la-front.jpg)

![](la-back.jpg)
