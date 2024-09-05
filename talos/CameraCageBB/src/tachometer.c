#include "tachometer.pio.h"

#include "hardware/pio.h"
#include "pico/stdlib.h"

#define CCBB_TACHOMETER_PIN 10

void getCCBBTachometerRPM(PIO pio, uint sm) {
    uint16_t ticks = pio_sm_get_blocking(pio, sm);
    printf("%d\n", ticks);
}

int main() {
    stdio_init_all();

    // Choose which PIO instance to use (there are two instances)
    PIO pio = pio0;

    // Our assembled program needs to be loaded into this PIO's instruction
    // memory. This SDK function will find a location (offset) in the
    // instruction memory where there is enough space for our program. We need
    // to remember this location!
    uint offset = pio_add_program(pio, &tachometer_program);

    // Find a free state machine on our chosen PIO (erroring if there are
    // none). Configure it to run our program, and start it, using the
    // helper function we included in our .pio file.
    uint sm = pio_claim_unused_sm(pio, true);
    tachometer_program_init(pio, sm, offset, CCBB_TACHOMETER_PIN);

    // The state machine is now running. Any value we push to its TX FIFO will
    // appear on the LED pin.
    while (true) {
        getCCBBTachometerRPM(pio, sm);
    }
}
