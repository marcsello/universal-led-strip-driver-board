//
// Created by marcsello on 2024.06.20..
//

#ifndef UNIVERSAL_LED_STRIP_DRIVER_BOARD_UART_H
#define UNIVERSAL_LED_STRIP_DRIVER_BOARD_UART_H

#include <stdint.h>

#if ENABLED_CHANNELS > 4 || ENABLED_CHANNELS < 1
#error "Configure 1-4 channels"
#endif

#define UART_BAUD 4800
#define UBBR_VAL ((F_CPU / (UART_BAUD * 16L)) - 1)

#define FULL_FRAME_LEN (uint8_t)(ENABLED_CHANNELS+2)
#define TAIL_BUF_LEN (uint8_t)(FULL_FRAME_LEN-1)


void init_uart(void);
uint8_t tx_ready(void);
void tx_frame(const uint8_t *full_frame);


#endif //UNIVERSAL_LED_STRIP_DRIVER_BOARD_UART_H
