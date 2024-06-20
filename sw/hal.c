//
// Created by marcsello on 2024.06.20..
//

#include <util/delay.h>
#include <avr/interrupt.h>

#include "hal.h"

void init_board(void) {
    // Init I/O pins
    DDRA = 0x01; // set PA0 as output
    PORTA = 0x01; // default PA0 -> PS-ON is high = turn PSU off; PA1 -> PG don't pull up, it's an inverted signal
    DDRB = 0x1e; // set PB1, PB2, PB3 and PB4 as outputs
    DDRD = 0x20; // set PD5 as output, the rest as input
    PORTD = 0x5c; // Pull up button inputs

    // Configure PWM outputs...
    // Timer 0
    TCCR0A = (1 << COM0A1) | (1 << COM0A0) | // Set OC0A on Compare Match, clear OC0A at TOP
             (1 << COM0B1) | (1 << COM0B0) | // Set OC0B on Compare Match, clear OC0B at TOP
             (1 << WGM01) | (1 << WGM00); // Fast PWM mode

    TCCR0B = (1 << CS01); // set clk/8 pre-scaler ~125kHz (to avoid audible range)

    OCR0A = 0xff; // inverted mode, set duty cycle to 0
    OCR0B = 0xff;

#if ENABLED_CHANNELS > 2
    // Timer 1
    TCCR1A = (1 << COM1A1) | (1 << COM1A0) | // Set OC1A/OC1B on Compare Match, clear OC1A/OC1B at TOP
             (1 << COM1B1) | (1 << COM1B0) |
             (1 << WGM10); // Fast PWM, 8-bit mode

    TCCR1B = (1 << WGM12) | // Fast PWM, 8-bit mode
             (1 << CS11); // set clk/8 pre-scaler ~125kHz (to avoid audible range)

    OCR1AL = 0xff; // inverted mode, set duty cycle to 0
    OCR1BL = 0xff;

#endif

}


_Noreturn void fault(uint8_t pattern) {
    // Set everything to the safe value and flash the fault led.
    // only way out is to RESET the board

    // Disable all interrupts
    cli();

    // set all pwm output to 0% duty cycle (this is not actually needed)
    OCR0B = 0xff;
    OCR0A = 0xff;
    OCR1AL = 0xff;
    OCR1BL = 0xff;

    // Disable PWM outputs
    TCCR0A = 0x00; // disconnect comparator from port
    TCCR1A = 0x00; // disconnect comparator from port

    // Set all (PWM) outputs to LOW
    PORTB &= 0x1c; // turn off PB2, PB3 and PB4, leave the rest as is
    PORTD &= 0xdf; // turn off PD5, leave the rest as is

    // Shut down the PSU
    PORTA |= 0x01; // PA0 -> PS-ON high = turn off PSU

    // enter the infinite loop
    while (1) {
        for (uint8_t i = 0; i < 8; i++) {
            if (pattern&(1<<i)) {
                PORTB |= 0x02; // fault led on
            } else {
                PORTB &= 0xFD; // fault led off
            }
            _delay_ms(125);
        }
    }
}

void set_pwm(uint8_t ch, uint8_t val) {
    uint8_t inv = 0xff - val; // invert value

    switch (ch) { // NOLINT(*-multiway-paths-covered)
        case 0:
            OCR0B = inv;
            return;
#if ENABLED_CHANNELS > 1
            case 1:
            OCR0A = inv;
            return;
#endif
#if ENABLED_CHANNELS > 2
            case 2:
            OCR1AL = inv; // we only need to set the low byte, because that timer is configured to run in 8bit mode
            return;
#endif
#if ENABLED_CHANNELS > 3
            case 3:
            OCR1BL = inv;
            return;
#endif
        default:
            fault(FAULT_PATTERN_LOGIC_ERR);
    }
}
