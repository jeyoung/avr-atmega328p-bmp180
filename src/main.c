#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include "bmp180.h"
#include "usi.h"

#define I2C DDRC
#define I2C_READ PINC
#define SCL PC5
#define SDA PC4

static const char *template = "AC1: %hd, AC2: %hd, AC3: %hd, AC4: %hu, AC5: %hu, AC6: %hu, B1: %hd, B2: %hd, MB: %hd, MC: %hd, MD: %hd, UT: %ld, UP: %ld, temperature: %ld\n";
static char message[12] = "\0";

int main(void)
{
    struct bmp180_measurements measurements = { 0 };

    usart_init();
    while (1) {
	bmp180_measure(&measurements);
	sprintf(message, template, measurements.ac1, measurements.ac2, measurements.ac3, measurements.ac4, measurements.ac5, measurements.ac6, measurements.b1, measurements.b2, measurements.mb, measurements.mc, measurements.md, measurements.ut, measurements.up, measurements.temperature);
	usart_send_data(message);
        delay_ms(1000);
    }
}

void delay_ms(uint16_t ms)
{
    while (--ms > 0)
        _delay_ms(1);
}
