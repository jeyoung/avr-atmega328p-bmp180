#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

#include "bmp180.h"

#ifndef I2C
#define I2C DDRC
#endif

#ifndef I2C_READ
#define I2C_READ PINC
#endif

#ifndef SCL
#define SCL PC5
#endif

#ifndef SDA
#define SDA PC4
#endif

int main(void)
{
    while (1) {
	bmp180_measure();
        delay_ms(1000);
    }
}

void delay_ms(uint16_t ms)
{
    while (--ms > 0)
        _delay_ms(1);
}
