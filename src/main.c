#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define I2C DDRC
#define SCL PC5
#define SDA PC4

#define PERIOD_TICKS (1000000/300000)
#define TICK _delay_us(PERIOD_TICKS);

static volatile enum { NONE, START, ADDRESS_WRITE, ACK_ADDRESS_WRITE, REGISTER_ADDRESS, ACK_REGISTER_ADDRESS, CONTROL_DATA, ACK_CONTROL_DATA, RESTART, ADDRESS_READ, ACK_ADDRESS_READ, MSB_READ, STOP } state = NONE;

static volatile uint8_t byte = 0;
static volatile uint8_t bit_counter = 0;
static volatile uint8_t ack_attempts = 0;

void delay_ms(uint16_t);

void i2c_init()
{
    I2C &= ~(1 << SDA);
    TICK
}

void i2c_start()
{
    I2C |= (1 << SDA);
    TICK
    I2C |= (1 << SCL);
}

uint8_t i2c_write()
{
    I2C |= (1 << SCL);
    if (byte & 0x80) {
	I2C &= ~(1 << SDA);
    } else {
	I2C |= (1 << SDA);
    }
    TICK
    I2C &= ~(1 << SCL);
    byte <<= 1;
    return --bit_counter;
}

uint8_t i2c_read()
{
    I2C |= (1 << SCL);
    I2C &= ~(1 << SDA);
    TICK
    I2C &= ~(1 << SCL);
    byte = (byte >> 1) | ((PINB & (1 << SDA)) << 7);
    return ++bit_counter;
}

uint8_t i2c_ack()
{
    I2C |= (1 << SCL);
    I2C &= ~(1 << SDA);
    TICK
    I2C &= ~(1 << SCL);
    return (PINB & (1 << SDA)) == 0;
}

void i2c_stop()
{
    I2C |= (1 << SCL);
    I2C |= (1 << SDA);
    I2C &= ~(1 << SCL);
    TICK
    I2C &= ~(1 << SDA);
}

void i2c()
{
    uint8_t running = 1;
    while (running) {
	switch (state) {
	    case NONE:
		i2c_init();
		state = START;
		break;

	    case START:
		i2c_start();
		state = ADDRESS_WRITE;
		break;

	    case ADDRESS_WRITE:
		if (bit_counter == 0) {
		    byte = 0xEE;
		    bit_counter = 8;
		} else if (i2c_write() == 0) {
		    ack_attempts = 0;
		    state = ACK_ADDRESS_WRITE;
		}
		break;

	    case ACK_ADDRESS_WRITE:
		if (i2c_ack()) {
		    state = REGISTER_ADDRESS;
#if 0
		} else if (++ack_attempts == 5) {
		    state = STOP;
#endif
		}
		break;

	    case REGISTER_ADDRESS:
		if (bit_counter == 0) {
		    byte = 0xF4;
		    bit_counter = 8;
		} else if (i2c_write() == 0) {
		    ack_attempts = 0;
		    state = ACK_REGISTER_ADDRESS;
		}
		break;

	    case ACK_REGISTER_ADDRESS:
		if (i2c_ack()) {
		    state = CONTROL_DATA;
#if 0
		} else if (++ack_attempts == 5) {
		    state = STOP;
#endif
		}
		break;

	    case CONTROL_DATA:
		if (bit_counter == 0) {
		    byte = 0x2E;
		    bit_counter = 8;
		} else if (i2c_write() == 0) {
		    ack_attempts = 0;
		    state = ACK_CONTROL_DATA;
		}
		break;

	    case ACK_CONTROL_DATA:
		if (i2c_ack() || ++ack_attempts == 5) {
		    state = STOP;
		}
		break;

	    case RESTART:
		i2c_start();
		state = ADDRESS_READ;
		break;
		
	    case ADDRESS_READ:
		if (bit_counter == 0) {
		    byte = 0xEF;
		    bit_counter = 8;
		} else if (i2c_write() == 0) {
		    ack_attempts = 0;
		    state = ACK_ADDRESS_READ;
		}
		break;

	    case ACK_ADDRESS_READ:
		if (i2c_ack()) {
		    bit_counter = 0;
		    state = MSB_READ;
#if 0
		} else if (++ack_attempts == 5) {
		    state = stop;
#endif
		}
		break;

	    case MSB_READ:
		if (i2c_read() == 8) {
		    state = STOP;
		}
		break;

	    case STOP:
		i2c_stop();
		running = 0;
		byte = 0;
		bit_counter = 0;
		ack_attempts = 0;
		break;
	}
    }
}

int main(void)
{
    /*
     * Set SCL and SDA pins to open-drain
     */
    I2C &= ~(1 << SCL) & ~(1 << SDA);
    I2C |=  (1 << SCL) |  (1 << SDA);
    I2C &= ~(1 << SCL) & ~(1 << SDA);

    while (1)
    {
	i2c();
        delay_ms(1000);
    }
}

void delay_ms(uint16_t ms)
{
    while (--ms > 0)
        _delay_ms(1);
}
