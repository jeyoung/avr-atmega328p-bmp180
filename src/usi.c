#include <avr/io.h>

#include "usi.h"

void usart_init(void)
{
    UBRR0H  = 0x00;
    UBRR0L  = 0x06;
    UCSR0B |= (1 << TXEN0);
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
}

void usart_send_byte(char byte)
{
    while (!(UCSR0A & (1 << UDRE0)))
	    ;
    UDR0 = byte;
}

void usart_send_data(const char *data)
{
    while (*data) {
	usart_send_byte(*data++);
    }
}
