#include <avr/io.h>
#include <util/delay.h>

#include "i2c.h"

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

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#ifndef BAUD_RATE
#define BAUD_RATE 1000000
#endif

#define PERIOD_TICKS (F_CPU/BAUD_RATE)
#define HOLD _delay_us(PERIOD_TICKS);


/*
 * Initialises the I2C
 */
void i2c_init()
{
    /*
     * Set SCL and SDA pins to open-drain
     */
    I2C &= ~(1 << SCL) & ~(1 << SDA);
    I2C |=  (1 << SCL) |  (1 << SDA);
    I2C &= ~(1 << SCL) & ~(1 << SDA);
}

/*
 * Starts an I2C communication
 */
void i2c_start()
{
    /*
     * If this tick is missing, things break!
     */
    I2C |= (1 << SCL);
    I2C &= ~(1 << SCL);

    I2C |= (1 << SDA);
    I2C |= (1 << SCL);
}

/*
 * Detects an ACK
 */
uint8_t i2c_ack()
{
    I2C |= (1 << SCL);
    I2C &= ~(1 << SDA);
    I2C &= ~(1 << SCL);
    return (I2C_READ & (1 << SDA)) == 0;
}

/*
 * Sends a ACKM
 */
void i2c_ackm()
{
    I2C |= (1 << SCL);
    I2C |= (1 << SDA);
    I2C &= ~(1 << SCL);
}

/*
 * Sends a NACKM
 */
void i2c_nackm()
{
    I2C |= (1 << SCL);
    I2C &= ~(1 << SDA);
    I2C &= ~(1 << SCL);
}

/*
 * Stops an I2C communication
 */
void i2c_stop()
{
    I2C &= ~(1 << SCL);
    I2C |= (1 << SCL);
    I2C |= (1 << SDA);
    I2C &= ~(1 << SCL);
    I2C &= ~(1 << SDA);
}

/*
 * Writes a byte to the I2C channel and updates the state of the application
 */
void i2c_write(struct i2c_write_data *data, enum i2c_state *state)
{
    switch (data->state) {
	case W_NONE:
	    break;

	case W_WRITE:
	    I2C |= (1 << SCL);
	    if (data->byte & 0x80) {
		I2C &= ~(1 << SDA);
	    } else {
		I2C |= (1 << SDA);
	    }
	    I2C &= ~(1 << SCL);

	    data->byte <<= 1;

	    if (--data->bit_counter == 0) {
		if (i2c_ack()) {
		    data->state = W_ACKS;
		} else {
		    data->state = W_ERROR;
		}
	    }
	    break;

	case W_ACKS:
	    data->state = W_NONE;
	    *state = data->success_state;
	    break;

	case W_ERROR:
	    data->state = W_NONE;
	    *state = data->error_state;
	    break;
    }
}

/*
 * Reads a byte from the I2C channel and updates the state of the application
 */
void i2c_read(struct i2c_read_data *data, enum i2c_state *state)
{
    switch (data->state) {
	case R_NONE:
	    break;

	case R_READ:
	    I2C &= ~(1 << SDA);
	    data->byte = 0;
	    data->state = R_READING;
	    break;

	case R_READING:
	    data->byte <<= 1;

	    I2C |= (1 << SCL);
	    I2C &= ~(1 << SCL);

	    if (I2C_READ & (1 << SDA)) {
		data->byte |= 0x01;
	    }

	    if (++data->bit_counter == 8) {
		if (data->send_nack) {
		    data->state = R_NACKM;
		} else {
		    data->state = R_ACKM;
		}
	    }
	    break;

	case R_NACKM:
	    i2c_nackm();
	    data->state = R_NONE;
	    *state = data->success_state;
	    break;

	case R_ACKM:
	    i2c_ackm();
	    data->state = R_NONE;
	    *state = data->success_state;
	    break;
    }
}



