#include <stdint.h>

#ifndef I2C_H
#define I2C_H

#ifndef I2C_STATE
#define I2C_STATE \
enum i2c_state { NONE, START, ADDRESS_WRITE, ADDRESS_READ, STOP };
#endif

/*
 * Represents the state of the application
 */
I2C_STATE

/*
 * Represents the state of the I2C write
 */
enum i2c_write_state { W_NONE, W_WRITE, W_ACKS, W_ERROR };

/*
 * Represents the state of the I2C read
 */
enum i2c_read_state  { R_NONE, R_READ, R_READING, R_ACKM, R_NACKM, R_ERROR };

/*
 * Represents the context data of the I2C write
 */
struct i2c_write_data {
    uint8_t byte;
    uint8_t bit_counter;
    enum i2c_write_state state;
    enum i2c_state success_state;
    enum i2c_state error_state;
};

/*
 * Represents the context data of the I2C read
 */
struct i2c_read_data {
    uint8_t byte;
    uint8_t bit_counter;
    uint8_t send_nack;
    enum i2c_read_state state;
    enum i2c_state success_state;
};

/*
 * Initialises the I2C
 */
void i2c_init();

/*
 * Starts an I2C communication
 */
void i2c_start();

/*
 * Detects an ACK
 */
uint8_t i2c_ack();

/*
 * Sends a ACKM
 */
void i2c_ackm();

/*
 * Sends a NACKM
 */
void i2c_nackm();

/*
 * Stops an I2C communication
 */
void i2c_stop();

/*
 * Writes a byte to the I2C channel and updates the state of the application
 */
void i2c_write(struct i2c_write_data *data, enum i2c_state *state);

/*
 * Reads a byte from the I2C channel and updates the state of the application
 */
void i2c_read(struct i2c_read_data *data, enum i2c_state *state);

#endif

