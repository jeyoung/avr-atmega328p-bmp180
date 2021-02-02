#include <avr/io.h>
#include <util/delay.h>

#include "bmp180.h"
#include "i2c.h"

#define OSS 3

#define CALIBRATION_MEASUREMENTS_STATE_CASE(CALIB_ID, MEASUREMENT_FIELD, TYPE, NEXT_CALIB_ID) \
		    case M_READ_##CALIB_ID##_MSB: \
			MEASUREMENT_FIELD = (TYPE) read_data.byte << 8; \
			measurements_state = M_READ_##CALIB_ID##_LSB; \
			break; \
		    case M_READ_##CALIB_ID##_LSB: \
			MEASUREMENT_FIELD |= (TYPE) read_data.byte; \
			measurements_state = M_READ_##NEXT_CALIB_ID##_MSB; \
			break; \

#define CALIBRATION_REGISTER_WRITE_CASE(CALIB_ID, REGISTER_MSB, REGISTER_LSB) \
	    case CALIB_ID##_MSB_REGISTER: \
		if (write_data.state == W_NONE) { \
		    write_data.byte = REGISTER_MSB; \
		    write_data.bit_counter = 8; \
		    write_data.success_state = RESTART; \
		    write_data.error_state = STOP; \
		    write_data.state = W_WRITE; \
		} \
		i2c_write(&write_data, &i2c_state); \
		break; \
	    case CALIB_ID##_LSB_REGISTER: \
		if (write_data.state == W_NONE) { \
		    write_data.byte = REGISTER_LSB; \
		    write_data.bit_counter = 8; \
		    write_data.success_state = RESTART; \
		    write_data.error_state = STOP; \
		    write_data.state = W_WRITE; \
		} \
		i2c_write(&write_data, &i2c_state); \
		break; \

#define CALIBRATION_SUCCESS_STATE_CASE(CALIB_ID) \
			case M_READ_##CALIB_ID##_MSB: \
			    write_data.success_state = CALIB_ID##_MSB_REGISTER; \
			    break; \
			case M_READ_##CALIB_ID##_LSB: \
			    write_data.success_state = CALIB_ID##_LSB_REGISTER; \
			    break; \

/*
 * Starts the I2C processing
 */
void bmp180_measure(struct bmp180_measurements *measurements)
{
    struct i2c_write_data write_data = { .state = W_NONE };
    struct i2c_read_data read_data = { .state = R_NONE };

    enum i2c_state i2c_state = NONE;
    enum measurements_state measurements_state = M_NONE;

    while (measurements_state != M_STOP) {
	switch (i2c_state) {
	    case NONE:
		i2c_init();
		i2c_state = START;
		break;

	    case START:
		/*
		 * Determine the next state based on the current state
		 */
		switch (measurements_state) {
		    case M_NONE:
			measurements_state = M_READ_ID;
			break;

		    case M_READ_ID:
			measurements_state = M_READ_AC1_MSB;
			break;

		    CALIBRATION_MEASUREMENTS_STATE_CASE(AC1, measurements->ac1, int16_t, AC2)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(AC2, measurements->ac2, int16_t, AC3)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(AC3, measurements->ac3, int16_t, AC4)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(AC4, measurements->ac4, uint16_t, AC5)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(AC5, measurements->ac5, uint16_t, AC6)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(AC6, measurements->ac6, uint16_t, B1)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(B1, measurements->b1, int16_t, B2)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(B2, measurements->b2, int16_t, MB)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(MB, measurements->mb, int16_t, MC)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(MC, measurements->mc, int16_t, MD)

		    case M_READ_MD_MSB:
			measurements->md = read_data.byte << 8;
			measurements_state = M_READ_MD_LSB;
			break;

		    case M_READ_MD_LSB:
			measurements->md |= read_data.byte;
			measurements_state = M_MEASURE_UT;
			break;

		    case M_MEASURE_UT:
			/*
			 * Wait 5ms before reading
			 */
			delay_ms(5);
			measurements_state = M_READ_UT_MSB;
			break;

		    case M_READ_UT_MSB:
			measurements->ut = (int32_t) read_data.byte << 8;
			measurements_state = M_READ_UT_LSB;
			break;

		    case M_READ_UT_LSB:
			measurements->ut |= (int32_t) read_data.byte;
			measurements_state = M_MEASURE_UP;
			break;

		    case M_MEASURE_UP:
			/*
			 * Wait 80ms before reading
			 */
			delay_ms(80);
			measurements_state = M_READ_UP_MSB;
			break;

		    case M_READ_UP_MSB:
			measurements->up = (int32_t) read_data.byte << 16;
			measurements_state = M_READ_UP_LSB;
			break;

		    case M_READ_UP_LSB:
			measurements->up |= (int32_t) read_data.byte << 8;
			measurements_state = M_READ_UP_XLSB;
			break;

		    case M_READ_UP_XLSB:
			measurements->up |= (int32_t) read_data.byte;
			measurements->up = measurements->up >> (8 - OSS);
			measurements_state = M_STOP;
			break;
		}

		if (measurements_state != M_STOP) {
		    i2c_start();
		    i2c_state = ADDRESS_WRITE;
		} else {
		    i2c_state = STOP;
		}
		break;

	    case ADDRESS_WRITE:
		if (write_data.state == W_NONE) {
		    write_data.byte = 0xEE;
		    write_data.bit_counter = 8;
		    switch (measurements_state) {
			case M_READ_ID:
			    write_data.success_state = ID_REGISTER;
			    break;

			case M_MEASURE_UT:
			case M_MEASURE_UP:
			    write_data.success_state = CONTROL_REGISTER;
			    break;

			case M_READ_UT_MSB:
			case M_READ_UP_MSB:
			    write_data.success_state = MSB_REGISTER;
			    break;

			case M_READ_UT_LSB:
			case M_READ_UP_LSB:
			    write_data.success_state = LSB_REGISTER;
			    break;

			case M_READ_UP_XLSB:
			    write_data.success_state = XLSB_REGISTER;
			    break;

			CALIBRATION_SUCCESS_STATE_CASE(AC1)
			CALIBRATION_SUCCESS_STATE_CASE(AC2)
			CALIBRATION_SUCCESS_STATE_CASE(AC3)
			CALIBRATION_SUCCESS_STATE_CASE(AC4)
			CALIBRATION_SUCCESS_STATE_CASE(AC5)
			CALIBRATION_SUCCESS_STATE_CASE(AC6)
			CALIBRATION_SUCCESS_STATE_CASE(B1)
			CALIBRATION_SUCCESS_STATE_CASE(B2)
			CALIBRATION_SUCCESS_STATE_CASE(MB)
			CALIBRATION_SUCCESS_STATE_CASE(MC)
			CALIBRATION_SUCCESS_STATE_CASE(MD)

			default:
			    write_data.success_state = STOP;
			    break;
		    }
		    write_data.error_state = STOP;
		    write_data.state = W_WRITE;
		}
		i2c_write(&write_data, &i2c_state);
		break;

	    case CONTROL_REGISTER:
		if (write_data.state == W_NONE) {
		    write_data.byte = 0xF4;
		    write_data.bit_counter = 8;
		    switch (measurements_state) {
			case M_MEASURE_UT:
			    write_data.success_state = MEASURE_UT;
			    break;

			case M_MEASURE_UP:
			    write_data.success_state = MEASURE_UP;
			    break;

			default:
			    write_data.success_state = STOP;
			    break;
		    }
		    write_data.error_state = STOP;
		    write_data.state = W_WRITE;
		}
		i2c_write(&write_data, &i2c_state);
		break;

	    case MEASURE_UT:
		if (write_data.state == W_NONE) {
		    write_data.byte = 0x2E;
		    write_data.bit_counter = 8;
		    write_data.success_state = STOP_START;
		    write_data.error_state = STOP;
		    write_data.state = W_WRITE;
		}
		i2c_write(&write_data, &i2c_state);
		break;

	    case MEASURE_UP:
		if (write_data.state == W_NONE) {
		    write_data.byte = 0xF4;
		    write_data.bit_counter = 8;
		    write_data.success_state = STOP_START;
		    write_data.error_state = STOP;
		    write_data.state = W_WRITE;
		}
		i2c_write(&write_data, &i2c_state);
		break;

	    case MSB_REGISTER:
		if (write_data.state == W_NONE) {
		    write_data.byte = 0xF6;
		    write_data.bit_counter = 8;
		    write_data.success_state = RESTART;
		    write_data.error_state = STOP;
		    write_data.state = W_WRITE;
		}
		i2c_write(&write_data, &i2c_state);
		break;

	    case LSB_REGISTER:
		if (write_data.state == W_NONE) {
		    write_data.byte = 0xF7;
		    write_data.bit_counter = 8;
		    write_data.success_state = RESTART;
		    write_data.error_state = STOP;
		    write_data.state = W_WRITE;
		}
		i2c_write(&write_data, &i2c_state);
		break;

	    case XLSB_REGISTER:
		if (write_data.state == W_NONE) {
		    write_data.byte = 0xF8;
		    write_data.bit_counter = 8;
		    write_data.success_state = RESTART;
		    write_data.error_state = STOP;
		    write_data.state = W_WRITE;
		}
		i2c_write(&write_data, &i2c_state);
		break;

	    case RESTART:
		i2c_start();
		i2c_state = ADDRESS_READ;
		break;
		
	    case ADDRESS_READ:
		if (write_data.state == W_NONE) {
		    write_data.byte = 0xEF;
		    write_data.bit_counter = 8;
		    switch (measurements_state) {
			case M_READ_ID:
			    write_data.success_state = ID_READ;
			    break;

			case M_READ_AC1_MSB:
			case M_READ_AC2_MSB:
			case M_READ_AC3_MSB:
			case M_READ_AC4_MSB:
			case M_READ_AC5_MSB:
			case M_READ_AC6_MSB:
			case M_READ_B1_MSB:
			case M_READ_B2_MSB:
			case M_READ_MB_MSB:
			case M_READ_MC_MSB:
			case M_READ_MD_MSB:
			case M_READ_UP_MSB:
			case M_READ_UT_MSB:
			    write_data.success_state = MSB_READ;
			    break;

			case M_READ_AC1_LSB:
			case M_READ_AC2_LSB:
			case M_READ_AC3_LSB:
			case M_READ_AC4_LSB:
			case M_READ_AC5_LSB:
			case M_READ_AC6_LSB:
			case M_READ_B1_LSB:
			case M_READ_B2_LSB:
			case M_READ_MB_LSB:
			case M_READ_MC_LSB:
			case M_READ_MD_LSB:
			case M_READ_UP_LSB:
			case M_READ_UT_LSB:
			    write_data.success_state = LSB_READ;
			    break;

			case M_READ_UP_XLSB:
			    write_data.success_state = XLSB_READ;
			    break;

			default:
			    write_data.success_state = STOP;
			    break;
		    }
		    write_data.error_state = STOP;
		    write_data.state = W_WRITE;
		}
		i2c_write(&write_data, &i2c_state);
		break;

	    case MSB_READ:
		if (read_data.state == R_NONE) {
		    read_data.bit_counter = 0;
		    read_data.send_nack = 1;
		    read_data.success_state = STOP_START;
		    read_data.state = R_READ;
		}
		i2c_read(&read_data, &i2c_state);
		break;

	    case LSB_READ:
		if (read_data.state == R_NONE) {
		    read_data.bit_counter = 0;
		    read_data.send_nack = 1;
		    read_data.success_state = STOP_START;
		    read_data.state = R_READ;
		}
		i2c_read(&read_data, &i2c_state);
		break;

	    case XLSB_READ:
		if (read_data.state == R_NONE) {
		    read_data.bit_counter = 0;
		    read_data.send_nack = 1;
		    read_data.success_state = STOP_START;
		    read_data.state = R_READ;
		}
		i2c_read(&read_data, &i2c_state);
		break;

	    case ID_REGISTER:
		if (write_data.state == W_NONE) {
		    write_data.byte = 0xD0;
		    write_data.bit_counter = 8;
		    write_data.success_state = RESTART;
		    write_data.error_state = STOP;
		    write_data.state = W_WRITE;
		}
		i2c_write(&write_data, &i2c_state);
		break;

	    case ID_READ:
		if (read_data.state == R_NONE) {
		    read_data.byte = 0;
		    read_data.bit_counter = 0;
		    read_data.send_nack = 1;
		    read_data.success_state = STOP_START;
		    read_data.state = R_READ;
		}
		i2c_read(&read_data, &i2c_state);
		break;

	    CALIBRATION_REGISTER_WRITE_CASE(AC1, 0xAA, 0xAB)
	    CALIBRATION_REGISTER_WRITE_CASE(AC2, 0xAC, 0xAD)
	    CALIBRATION_REGISTER_WRITE_CASE(AC3, 0xAE, 0xAF)
	    CALIBRATION_REGISTER_WRITE_CASE(AC4, 0xB0, 0xB1)
	    CALIBRATION_REGISTER_WRITE_CASE(AC5, 0xB2, 0xB3)
	    CALIBRATION_REGISTER_WRITE_CASE(AC6, 0xB4, 0xB5)
	    CALIBRATION_REGISTER_WRITE_CASE(B1, 0xB6, 0xB7)
	    CALIBRATION_REGISTER_WRITE_CASE(B2, 0xB8, 0xB9)
	    CALIBRATION_REGISTER_WRITE_CASE(MB, 0xBA, 0xBB)
	    CALIBRATION_REGISTER_WRITE_CASE(MC, 0xBC, 0xBD)
	    CALIBRATION_REGISTER_WRITE_CASE(MD, 0xBE, 0xBF)

	    case STOP:
		i2c_stop();
		break;

	    case STOP_START:
		i2c_stop();
		i2c_state = START;
		break;
	}
    }

    measurements_state = M_NONE;
    bmp180_calculate(measurements);
}

void bmp180_calculate(struct bmp180_measurements *measurements)
{
#if 0
    measurements->ac1 = 408;
    measurements->ac2 = -72;
    measurements->ac3 = -14383;
    measurements->ac4 = 32741;
    measurements->ac5 = 32757;
    measurements->ac6 = 23153;
    measurements->b1 = 6190;
    measurements->b2 = 4;
    measurements->mb = -32768;
    measurements->mc = -8711;
    measurements->md = 2868;
    measurements->ut = 27898;
    measurements->up = 23843;
#endif

    /*
     * Calculate temperature
     */
    int32_t x1 = (measurements->ut - measurements->ac6) * measurements->ac5 / pow(2, 15);
    int32_t x2 = measurements->mc * pow(2, 11) / (x1 + measurements->md);
    int32_t b5 = x1 + x2;
    measurements->temperature = (b5 + 8) / pow(2, 4);

    /*
     * Calculate pressure
     */
    int32_t b6 = b5 - 4000;
    x1 = (measurements->b2 * (b6 * b6 / pow(2, 12))) / pow(2, 11);
    x2 = measurements->ac2 * b6 / pow(2, 11);
    int32_t x3 = x1 + x2;
    int32_t b3 = (((measurements->ac1 * 4 + x3) << OSS) + 2) / 4;
    x1 = measurements->ac3 * b6 / pow(2, 13);
    x2 = (measurements->b1 * (b6 * b6 / pow(2, 12))) / pow(2, 16);
    x3 = (x1 + x2 + 2) / pow(2, 2);
    uint32_t b4 = measurements->ac4 * (uint32_t) (x3 + 32768) / pow(2, 15);
    uint32_t b7 = ((uint32_t) measurements->up - b3) * (50000 >> OSS);
    if (b7 < 0x80000000) {
	measurements->pressure = (b7 * 2) / b4;
    } else {
	measurements->pressure = (b7 / b4) * 2;
    }
    x1 = (measurements->pressure / pow(2, 8)) * (measurements->pressure / pow(2, 8));
    x1 = (x1 * 3038) / pow(2, 16);
    x2 = (-7357 * measurements->pressure) / pow(2, 16);
    measurements->pressure += (x1 + x2 + 3791) / pow(2, 4);
}
