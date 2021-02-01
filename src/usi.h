#ifndef USI_H
#define USI_H

void usart_init(void);

void usart_send_byte(char byte);

void usart_send_data(const char *data);

#endif
