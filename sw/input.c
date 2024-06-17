//
// Created by marcsello on 2024.06.16..
//
#include <stdint.h>
#include <avr/io.h>

uint8_t cyc_since_last_change = 0x00;
uint8_t last_state = 0x00;
uint8_t stable_state = 0x00;
uint8_t input_mask = 0xff;

inline uint8_t map_input(void) {
    // We map PD2, PD3, PD4 and PD6 to the low 4 bits of the int
    return ((PIND & 0x1c) >> 2) | ((PIND & 0x40) >> 3); // <- This is double flipped, reed relays are closed when the door is closed, and open when opened, since those pins are pulled up, 1 = door open, 0 = door closed
}

uint8_t read_input(void) { // highly magic, sleep-free de-bounce. Sadly, the actual de-bouncing depends on the "speed" of the main loop.
    uint8_t current_state = map_input();
    if (current_state != last_state) {
        cyc_since_last_change = 0x00;
        last_state = current_state;
    } else {
        if (cyc_since_last_change == 150) {

            input_mask |= stable_state^last_state; // set back bit to 1 where the change happened

            stable_state = last_state;
            cyc_since_last_change = 0xff;
        } else if (cyc_since_last_change < 150) {
            cyc_since_last_change++;
        }
    }
    return stable_state & input_mask;
}

void mask_input(uint8_t mask) { // mask considers the masked pins as low, until they change state...
    input_mask &= (uint8_t)~mask; // invert it, so set bits will become zero, and will produce zero in the end
}
