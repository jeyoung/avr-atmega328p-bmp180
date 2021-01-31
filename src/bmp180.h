#include <stdint.h>

#ifndef BMP180_H
#define BMP180_H

#ifdef I2C_STATE
#undef I2C_STATE
#endif

#define I2C_STATE \
enum i2c_state { NONE, START, ADDRESS_WRITE, AC1_MSB_REGISTER, AC1_LSB_REGISTER, AC2_MSB_REGISTER, AC2_LSB_REGISTER, AC3_MSB_REGISTER, AC3_LSB_REGISTER, AC4_MSB_REGISTER, AC4_LSB_REGISTER, AC5_MSB_REGISTER, AC5_LSB_REGISTER, AC6_MSB_REGISTER, AC6_LSB_REGISTER, B1_MSB_REGISTER, B1_LSB_REGISTER, B2_MSB_REGISTER, B2_LSB_REGISTER, MB_MSB_REGISTER, MB_LSB_REGISTER, MC_MSB_REGISTER, MC_LSB_REGISTER, MD_MSB_REGISTER, MD_LSB_REGISTER, CONTROL_REGISTER, MEASURE_UT, MEASURE_UP, RESTART, MSB_REGISTER, LSB_REGISTER, ADDRESS_READ, MSB_READ, LSB_READ, ID_REGISTER, ID_READ, RESULT, STOP };

#include "i2c.h"

/*
 * Represents the state of the BMP180 measurements
 */
enum measurements_state { M_NONE, M_READ_ID, M_READ_AC1_MSB, M_READ_AC1_LSB, M_READ_AC2_MSB, M_READ_AC2_LSB, M_READ_AC3_MSB, M_READ_AC3_LSB, M_READ_AC4_MSB, M_READ_AC4_LSB, M_READ_AC5_MSB, M_READ_AC5_LSB, M_READ_AC6_MSB, M_READ_AC6_LSB, M_READ_B1_MSB, M_READ_B1_LSB, M_READ_B2_MSB, M_READ_B2_LSB, M_READ_MB_MSB, M_READ_MB_LSB, M_READ_MC_MSB, M_READ_MC_LSB, M_READ_MD_MSB, M_READ_MD_LSB, M_MEASURE_UP, M_MEASURE_UT, M_READ_UP_MSB, M_READ_UP_LSB, M_READ_UT_MSB, M_READ_UT_LSB, M_RESULT, M_STOP };

/*
 * Represents a set of BMP180 measurements
 */
struct bmp180_measurements {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    int16_t ac4;
    int16_t ac5;
    int16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
    int16_t ut;
    int16_t up;
    int8_t temperature;
};

/*
 * Starts the BMP180 measurements
 */
void bmp180_measure();

/*
 * Delays processing by specified milliseconds
 */
void delay_ms(uint16_t);

#endif
