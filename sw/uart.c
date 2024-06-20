//
// Created by marcsello on 2024.06.20..
//

#include <avr/interrupt.h>
#include <avr/io.h>

#include "uart.h"

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

uint8_t tx_ready(void) {
    return tx_tail_ptr == 0xff;
}

void tx_frame(const uint8_t *full_frame) {
    if (tx_tail_ptr != 0xff) {
        return; // drop frame
    }

    tx_tail_ptr = 0;
    for (uint8_t i = 0; i < TAIL_BUF_LEN; i++) {
        tx_frame_tail_buf[i] = full_frame[i+1];
    }

    while(!(UCSRA & (1<<UDRE)));
    UDR = full_frame[0]; // start transmission
    UCSRB |= (1 << UDRIE); // Enable UDRE interrupt
}


void init_uart(void) {
    // UART
    UCSRA = 0x00;
    UCSRB = (1<<RXEN) | (1<<TXEN); // enable transmitter and receiver
    UCSRC = (1<<UPM1) | (1<<UPM0) | (1<<UCSZ1) | (1<<UCSZ0); // enable odd parity, set 8-bit frame size

    // set baud rate
    UBRRH = (uint8_t)(UBBR_VAL >> 8);
    UBRRL = (uint8_t)(UBBR_VAL);
}