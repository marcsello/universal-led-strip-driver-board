//
// Created by marcsello on 2024.06.20..
//

#ifndef UNIVERSAL_LED_STRIP_DRIVER_BOARD_HAL_H
#define UNIVERSAL_LED_STRIP_DRIVER_BOARD_HAL_H

#include <stdint.h>
#include <avr/io.h>

#if ENABLED_CHANNELS > 4 || ENABLED_CHANNELS < 1
#error "Configure 1-4 channels"
#endif

#define FAULT_PATTERN_PSU_ERR 0x0f // slow blink
#define FAULT_PATTERN_LOGIC_ERR 0x55 // fast blink

void init_board(void);
_Noreturn void fault(uint8_t pattern);
void set_pwm(uint8_t ch, uint8_t val);

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

#endif //UNIVERSAL_LED_STRIP_DRIVER_BOARD_HAL_H
