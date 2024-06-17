#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "input.h"


void init_board(void) {
    // no interrupts while initializing
    cli();

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

    // Timer 1
    TCCR1A = (1 << COM1A1) | (1 << COM1A0) | // Set OC1A/OC1B on Compare Match, clear OC1A/OC1B at TOP
             (1 << COM1B1) | (1 << COM1B0) |
             (1 << WGM10); // Fast PWM, 8-bit mode

    TCCR1B = (1 << WGM12) | // Fast PWM, 8-bit mode
             (1 << CS11); // set clk/8 pre-scaler ~125kHz (to avoid audible range)

    OCR1AL = 0xff; // inverted mode, set duty cycle to 0
    OCR1BL = 0xff;

    // Configure watchdog
    WDTCSR = (1 << WDIE); // interrupt enable, pre-scaler set to 2 = 16ms

    // enable interrupts
    sei();
}

#define FAULT_PATTERN_PSU_ERR 0x0f // slow blink
#define FAULT_PATTERN_LOGIC_ERR 0x55 // fast blink
_Noreturn void fault(uint8_t pattern) {
    // Set everything to the safe value and flash the fault led.
    // only way out is to RESET the board

    // Disable all interrupts
    cli();

    // set all pwm output to 0 duty cycle (this is not actually needed)
    set_pwm(0, 0);
    set_pwm(1, 0);
    set_pwm(2, 0);
    set_pwm(3, 0);

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
    switch (ch) {
        case 0:
            OCR0B = inv;
            return;
        case 1:
            OCR0A = inv;
            return;
        case 2:
            OCR1AL = inv; // we only need to set the low byte, because that timer is configured to run in 8bit mode
            return;
        case 3:
            OCR1BL = inv;
            return;
        default:
            // ignore
            return;
    }
}

inline static void set_psu(uint8_t val) {
    if (val) {
        PORTA &= 0xfe; // pull PA0/PS-ON to ground = turn on PSU
    } else {
        PORTA |= 0x01; // pull PA0/PS-ON HIGH = turn off PSU
    }
}

inline static uint8_t psu_pg(void) {
    return PINA & 0x02 // PA1: Power good: high active
}

volatile uint8_t timer_flag = 0x00; // msb = ticks counter enable, lsb = tick indicator
volatile uint16_t ticks = 0x0000;

ISR(WDT_OVERFLOW_vect) {
    timer_flag |= 0x01;
    if (timer_flag & 0x80) {
        ticks++;
    }
}

ISR(__vector_default) { // all unhandled interrupts
    fault(FAULT_PATTERN_LOGIC_ERR);
}

#define STATE_STANDBY 1 // idle, waiting for something to happen
#define STATE_START 2 // prepare stuff, send startup to psu
#define STATE_PSU_WARMUP 3 // wait for PG to go high
#define STATE_ACTIVE 4 // update outputs based on inputs, wait for timeouts
#define STATE_STOP 5 // prepare shutdown, send shutdown to psu
#define STATE_PSU_SHUTDOWN 6 // wait for PG to go low
#define STATE_GO_STANDBY 7 // prepare stuff to go back to standby

int main(void) {
    init_board();

    uint8_t state = STATE_STANDBY;

    uint8_t levels[4] = {0, 0, 0, 0};
    uint8_t last_input = 0x00;

    while (1) {
        if (ticks >= 65535) {
            // the tick counter should never go this high...
            fault(FAULT_PATTERN_LOGIC_ERR);
        }
        switch (state) {
            case STATE_STANDBY: {
                uint8_t input = read_input();
                if (input) {
                    state = STATE_START;
                }
                if (psu_pg()) { // PG should be low, if hi than it's a fault
                    fault(FAULT_PATTERN_PSU_ERR);
                }
            }
                break;
            case STATE_START: {
                ticks = 0; // reset tick counter
                timer_flag |= 0x80; // enable tick counter

                // zero out levels, in case they still have some garbage data...
                for (uint8_t i = 0; i < 4; i++) {
                    if (levels[i] > 0) {
                        levels[i] = 0;
                        set_pwm(i, 0);
                    }
                }

                set_psu(0x01); // turn on PSU
                state = STATE_PSU_WARMUP;
            }
                break;
            case STATE_PSU_WARMUP: {

                if (psu_pg()) { // wait for power good
                    ticks = 0; // reset tick counter
                    state = STATE_ACTIVE;
                } else if (ticks > 32) { // ~500ms (512ms)
                    fault(FAULT_PATTERN_PSU_ERR); // PSU could not start up in time
                }

            }
                break;
            case STATE_ACTIVE: {
                uint8_t input = read_input();

                // reset tick counter when some state changes...
                if (input != last_input) {
                    // reset ticks when something changes
                    ticks = 0;
                }
                last_input = input;

                // check if this loop is a "tick" loop
                uint8_t tick = 0x00;
                if (timer_flag & 0x01) {
                    tick = 0x01;
                    timer_flag &= 0xfe; // set the first bit to zero
                }

                for (uint8_t i = 0; i < 4; i++) {
                    uint8_t changed = 0x00;
                    if (state & (1 << i)) {
                        if ((levels[i] < 0xff) && tick) {
                            levels[i]++;
                            changed = 0x01;
                        }
                    } else {
                        if (levels[i] > 0) {
                            levels[i] = 0;
                            changed = 0x01;
                        }
                    }
                    if (changed) {
                        set_pwm(i, levels[i]);
                    }
                }

                if (input) {
                    // any door open
                    if (ticks > 37500) { // ~ 10 min
                        mask_input(input);
                        state = STATE_STOP;
                    }
                } else {
                    // all doors closed
                    if (ticks > 1875) { // ~ 30sec
                        // first bit = this loop is a tick
                        // don't need to mask... all doors should be closed
                        state = STATE_STOP;
                    }
                }

                if (!psu_pg()) { // PG should be high, if it's low, than this is a fault
                    fault(FAULT_PATTERN_PSU_ERR);
                }

            }
                break;
            case STATE_STOP: {
                ticks = 0; // reset tick counter

                // zero out levels, in case they still have some garbage data...
                for (uint8_t i = 0; i < 4; i++) {
                    if (levels[i] > 0) {
                        levels[i] = 0;
                        set_pwm(i, 0);
                    }
                }

                set_psu(0x00); // turn off PSU
                state = STATE_PSU_SHUTDOWN;
            }
                break;
            case STATE_PSU_SHUTDOWN: {

                if (!psu_pg()) { // wait for power good off
                    ticks = 0; // reset tick counter
                    state = STATE_GO_STANDBY;
                } else if (ticks > 32) { // ~500ms (512ms)
                    fault(FAULT_PATTERN_PSU_ERR); // PSU could not shut down in time
                }

            }
                break;
            case STATE_GO_STANDBY: {
                ticks = 0; // reset tick counter
                timer_flag &= 0x7F; // disable tick counter

                state = STATE_STANDBY;
            }
                break;
            default:
                fault(FAULT_PATTERN_LOGIC_ERR);
        }
    }
}
