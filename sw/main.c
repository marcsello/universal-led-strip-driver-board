#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "input.h"

#ifndef ENABLED_CHANNELS
#define ENABLED_CHANNELS 4
#endif

#if ENABLED_CHANNELS > 4 || ENABLED_CHANNELS < 1
#error "Configure 1-4 channels"
#endif

#define UART_BAUD 4800
#define UBBR_VAL ((F_CPU / (UART_BAUD * 16L)) - 1)


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

    // Configure watchdog timer as interrupt source
    WDTCSR = (1 << WDIE); // interrupt enable, pre-scaler set to 2 = 16ms

    // UART
    UCSRA = 0x00;
    UCSRB = (1<<RXEN) | (1<<TXEN); // enable transmitter and receiver
    UCSRC = (1<<UPM1) | (1<<UPM0) | (1<<UCSZ1) | (1<<UCSZ0); // enable odd parity, set 8-bit frame size

    // set baud rate
    UBRRH = (uint8_t)(UBBR_VAL >> 8);
    UBRRL = (uint8_t)(UBBR_VAL);

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
    switch (ch) {
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

inline static void set_psu(uint8_t val) {
    if (val) {
        PORTA &= 0xfe; // pull PA0/PS-ON to ground = turn on PSU
    } else {
        PORTA |= 0x01; // pull PA0/PS-ON HIGH = turn off PSU
    }
}

inline static uint8_t psu_pg(void) {
    return PINA & 0x02; // PA1: Power good: high active
}

static volatile uint8_t timer_flag = 0x00; // msb = ticks counter enable, lsb+1 = transmit metrics, lsb = tick indicator
static volatile uint16_t ticks = 0x0000;

ISR(WDT_OVERFLOW_vect) { // our "timer"
    timer_flag |= 0x03;
    if (timer_flag & 0x80) {
        ticks++;
    }
}

#define FULL_FRAME_LEN (uint8_t)(ENABLED_CHANNELS+2)
#define TAIL_BUF_LEN (uint8_t)(FULL_FRAME_LEN-1)

static volatile uint8_t tx_frame_tail_buf[TAIL_BUF_LEN]; // the first byte sent immediately, this is only the tail
static volatile uint8_t tx_tail_ptr = 0xff;

ISR(USART_UDRE_vect) { // this is a quite lame interrupt as it always triggers when the corresponding flag is set
    if (tx_tail_ptr < TAIL_BUF_LEN) {
        UDR = tx_frame_tail_buf[tx_tail_ptr++];
    } else {
        UCSRB &= ~(1 << UDRIE); // this interrupt has to be disabled, or it will be generated constantly
        tx_tail_ptr = 0xff; // indicate send ready condition
    }
}

void tx_frame(const uint8_t *full_frame) {
    while(!(UCSRA & (1<<UDRE)));

    tx_tail_ptr = 0;
    for (uint8_t i = 0; i < TAIL_BUF_LEN; i++) {
        tx_frame_tail_buf[i] = full_frame[i+1];
    }
    UDR = full_frame[0]; // start transmission
    UCSRB |= (1 << UDRIE); // Enable UDRE interrupt
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

    uint8_t levels[ENABLED_CHANNELS];
    for (uint8_t i = 0; i < ENABLED_CHANNELS; i++) {
        levels[i] = 0;
    }

    uint8_t last_input = 0x00;

    while (1) {

        // timer sanity check
        if (ticks >= 65535) {
            // the tick counter should never go this high...
            fault(FAULT_PATTERN_LOGIC_ERR);
        }

        // transmit metrics
        if ((timer_flag & 0x02) && (tx_tail_ptr == 0xff)) {
            timer_flag &= 0xfd;

            uint8_t status = (state<<4) | last_input; // upper 4 bit: current status, lower 4 bit: inputs
            uint8_t frame[FULL_FRAME_LEN] = {0x55, status}; // 0x55 is the preamble
            for (uint8_t i = 0; i<ENABLED_CHANNELS; i++) {
                frame[i+2] = levels[i];
            }
            tx_frame(frame);
        }

        // main state machine
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
                for (uint8_t i = 0; i < ENABLED_CHANNELS; i++) {
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

                for (uint8_t i = 0; i < ENABLED_CHANNELS; i++) {
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
                last_input = 0x00; // reset last input indicator
                ticks = 0; // reset tick counter

                // zero out levels, in case they still have some garbage data...
                for (uint8_t i = 0; i < ENABLED_CHANNELS; i++) {
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
