#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "input.h"
#include "uart.h"
#include "hal.h"

#if ENABLED_CHANNELS > 4 || ENABLED_CHANNELS < 1
#error "Configure 1-4 channels"
#endif

#define STATE_STANDBY 1 // idle, waiting for something to happen
#define STATE_START 2 // prepare stuff, send startup to psu
#define STATE_PSU_WARMUP 3 // wait for PG to go high
#define STATE_ACTIVE 4 // update outputs based on inputs, wait for timeouts
#define STATE_STOP 5 // prepare shutdown, send shutdown to psu
#define STATE_PSU_SHUTDOWN 6 // wait for PG to go low
#define STATE_GO_STANDBY 7 // prepare stuff to go back to standby


static volatile uint8_t timer_flag = 0x00; // msb = ticks counter enable, lsb+1 = transmit metrics, lsb = tick indicator
static volatile uint16_t ticks = 0x0000;
static volatile uint8_t metrics_timer = 0x00;

ISR(WDT_OVERFLOW_vect) { // our "timer"
    timer_flag |= 0x01;
    metrics_timer++;
    if (timer_flag & 0x80) {
        ticks++;
    }
}


inline static void init(void) {
    cli();
    init_board();
    init_uart();

    // Configure watchdog timer as interrupt source
    WDTCSR = (1 << WDIE); // interrupt enable, pre-scaler set to 2 = 16ms

    sei();
}

inline static void transmit_metrics(uint8_t state, uint8_t input, const uint8_t *levels) {
    uint8_t status = (state << 4) | input; // upper 4 bit: current status, lower 4 bit: inputs
    uint8_t frame[FULL_FRAME_LEN] = {0x55, status}; // 0x55 is the preamble
    for (uint8_t i = 0; i < ENABLED_CHANNELS; i++) {
        frame[i + 2] = levels[i];
    }
    tx_frame(frame);
}

int main(void) {
    init();

    uint8_t state = STATE_STANDBY;

    uint8_t levels[ENABLED_CHANNELS];
    for (uint8_t i = 0; i < ENABLED_CHANNELS; i++) {
        levels[i] = 0;
    }

    uint8_t last_input = 0x00;

    while (1) {

        // timer sanity check
        if (ticks >= 65534 || metrics_timer > 254) {
            // when counters go this high... something must have gone wrong...
            fault(FAULT_PATTERN_LOGIC_ERR);
        }

        // read input in every loop
        uint8_t input = read_input();

        // transmit metrics
        if ((metrics_timer > 8) && tx_ready()) {
            metrics_timer = 0;
            transmit_metrics(state, input, levels);
        }

        // main state machine
        switch (state) {
            case STATE_STANDBY: {
                if (input) { // any input is pulled high = contact opened = door opened
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


ISR(__vector_default) { // all unhandled interrupts
    fault(FAULT_PATTERN_LOGIC_ERR);
}
