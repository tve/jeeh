**Explorations using the Nucleo-G474RE or WeAct-G474CE board.**

- Nucleo: LED = PA5 - `pio run -e blink -t upload`
- WeAct: LED = PC6 - `pio run -e blink-wa -t upload`

Examples in this area:

- **`blink`** - blink the on-board LED
- **`hello`** - print some text on the serial port, using the dma-uart driver
- **`alive`** - keep the watchdog alive with a background timer
