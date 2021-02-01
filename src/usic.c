#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#ifndef USI_H
#define USI_H

#ifndef RX
#define RX  PB0
#endif

#ifndef TX
#define TX  PB1
#endif

#ifndef STATUS
#define STATUS  USISR
#endif

#ifndef CONTROL
#define CONTROL USICR
#endif

#ifndef DATA
#define DATA    USIDR
#endif

volatile uint8_t second_byte_sent = 0;

/*
 * The ATtiny85 has an 8-bit data register, but we need to send a total of 10
 * bits (8 bits of data, plus a start bit at the beginning and a stop bit at
 * the end). To achieve this, we must put the first 8 bits (1x start bit + 7x
 * data bits) in the data register and must keep the remaining bits (1x data
 * bit + 1x stop bit) in memory to be sent later.
 * After the first byte in the data register is sent, the second byte of data
 * with just 2 significant bits is put in the data register to be sent.
 */
char second_byte;

void delay_ms(uint16_t);

ISR(USI_OVF_vect)
{
    /*
     * This interrupt is triggered when the bit counter overflows.
     * If it is not the overflow for the second byte, send the data for the
     * second byte and set the byte counter to overflow after the next two bits
     * are sent.
     * Otherwise, set the TX port to HIGH and turn off USI.
     */
    if (!second_byte_sent) {
	second_byte_sent = 1;
	DATA = second_byte;

	/*
	 * Set the counter value to 0x0E so that it overflows after the next
	 * two bits (16 - 0x0E) are sent.
	 */
	 STATUS |= 0x0E;
    } else {
	PORTB |= (1 << TX);
	CONTROL = 0;
    }

    /*
     * Clear the overflow interrupt flag so that the next interrupt can be
     * triggered
     */
    STATUS |= (1 << USIOIF);
}

uint8_t reverse(uint8_t b)
{
    /*
     * Reverse the bits of the given byte.
     */
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

void init(void)
{
    /*
     * Set the LED and TX to output
     */
    DDRB |= (1 << LED) | (1 << TX);
    
    /*
     * Set the Timer/Counter 0 to CTC mode. In this mode, it will overflow when
     * it reaches the value in OCR0A.
     */
    TCCR0A |= (1 << WGM01);

    /*
     * Disable pre-scaling
     */
    TCCR0B |= (1 << CS00);

    /*
     * Set the overflow value to 0x68. At a clock frequency of 1MHz, this
     * yields 9600bps.
     */
    OCR0A |= 0x68; 

    sei();
}

void send_byte(uint8_t byte)
{
    init();

    while (1) {
	/*
	 * Set the TX port to high
	 */
	PORTB |= 1 << TX;

	/*
	 * Reset the 'second byte sent' flag
	 */
	second_byte_sent = 0;

	/*
	 * The data register sends the most significant bit (MSB) to the TX
	 * port first, but UART requires the least signficant bit (LSB) to be
	 * sent first. So we have to reverse the bits in the data byte before
	 * putting it in the data register.
	 */
	uint8_t reversed = reverse(byte);

	/*
	 * Put the first 8 bits in the data register and hold the remaining two
	 * bits in memory so that they can be sent afterwards.
	 * The first bit to be sent out must be the start LOW bit, therefore
	 * shift the data bits right so that the most significant bit generates
	 * a LOW on the TX port.
	 * The last bit to be sent out must be the stop HIGH bit, and there is
	 * only one remaining data bit to be sent out after the first 7 data
	 * bits have been put in the data register; therefore, shift the last
	 * bit to the MSB position and fill the rest with 1s.
	 */
	DATA = reversed >> 1;
	second_byte = (reversed << 7) | (0xFF >> 1);

	/*
	 * Set the counter value to 0x08 (16 - 8) so that it overflows after 8
	 * data bits have been sent.
	 */
	STATUS |= 0x08;

	/*
	 * Enable the USI, setting it to use Timer/Counter 0 as the clock
	 */
	CONTROL |= (1 << USIOIE) | (1 << USICS0) | (0 << USIWM1) | (1 << USIWM0);

	/*
	 * Clear the overflow interrupt flag so that the next interrupt can be
	 * triggered
	 */
	STATUS |= (1 << USIOIF);

	/*
	 * Start the timer counter
	 */
	TCNT0 = 0;
    }
}

void delay_ms(uint16_t ms)
{
    while (--ms > 0)
        _delay_ms(1);
}


#endif
