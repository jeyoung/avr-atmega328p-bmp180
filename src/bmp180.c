#include <avr/io.h>
#include <util/delay.h>

#include "bmp180.h"
#include "i2c.h"

#define PERIOD_TICKS (F_CPU/BAUD_RATE)
#define HOLD _delay_us(PERIOD_TICKS);

#define CALIBRATION_MEASUREMENTS_STATE_CASE(CALIB_ID, MEASUREMENT_FIELD, NEXT_CALIB_ID) \
		    case M_READ_##CALIB_ID##_MSB: \
			MEASUREMENT_FIELD = read_data.byte << 8; \
			measurements_state = M_READ_##CALIB_ID##_LSB; \
			break; \
		    case M_READ_##CALIB_ID##_LSB: \
			MEASUREMENT_FIELD |= read_data.byte; \
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
void bmp180_measure()
{
    struct i2c_write_data write_data = { .state = W_NONE };
    struct i2c_read_data read_data = { .state = R_NONE };

    struct bmp180_measurements measurements = { 0 };

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

		    CALIBRATION_MEASUREMENTS_STATE_CASE(AC1, measurements.ac1, AC2)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(AC2, measurements.ac2, AC3)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(AC3, measurements.ac3, AC4)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(AC4, measurements.ac4, AC5)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(AC5, measurements.ac5, AC6)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(AC6, measurements.ac6, B1)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(B1, measurements.b1, B2)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(B2, measurements.b2, MB)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(MB, measurements.mb, MC)
		    CALIBRATION_MEASUREMENTS_STATE_CASE(MC, measurements.mc, MD)

		    case M_READ_MD_MSB:
			measurements.md = read_data.byte << 8;
			measurements_state = M_READ_MD_LSB;
			break;

		    case M_READ_MD_LSB:
			measurements.md |= read_data.byte;
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
			measurements.ut = read_data.byte << 8;
			measurements_state = M_READ_UT_LSB;
			break;

		    case M_READ_UT_LSB:
			measurements.ut |= read_data.byte;
			measurements_state = M_MEASURE_UP;

			/*
			 * TODO (EY) Move this somewhere else
			 */
			int32_t x1 = ((int32_t) measurements.ut - (int32_t) measurements.ac6) * ((int32_t) measurements.ac5 >> 15);
			int32_t x2 = ((int32_t) measurements.mc << 11) / (x1 + (int32_t) measurements.md);
			int32_t b5 = x1 + x2;
			measurements.temperature = ((b5 + 8) >> 4);

			break;

		    case M_MEASURE_UP:
			/*
			 * Wait 5ms before reading
			 */
			delay_ms(5);
			measurements_state = M_READ_UP_MSB;
			break;

		    case M_READ_UP_MSB:
			measurements.up = read_data.byte << 8;
			measurements_state = M_READ_UP_LSB;
			break;

		    case M_READ_UP_LSB:
			measurements.up |= read_data.byte;
			measurements_state = M_RESULT;
			break;

		    case M_RESULT:
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

			case M_RESULT:
			    write_data.success_state = RESULT;
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
		    write_data.success_state = START;
		    write_data.error_state = STOP;
		    write_data.state = W_WRITE;
		}
		i2c_write(&write_data, &i2c_state);
		break;

	    case MEASURE_UP:
		if (write_data.state == W_NONE) {
		    write_data.byte = 0x34;
		    write_data.bit_counter = 8;
		    write_data.success_state = START;
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
		    read_data.success_state = START;
		    read_data.state = R_READ;
		}
		i2c_read(&read_data, &i2c_state);
		break;

	    case LSB_READ:
		if (read_data.state == R_NONE) {
		    read_data.bit_counter = 0;
		    read_data.send_nack = 1;
		    read_data.success_state = START;
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
		    read_data.success_state = START;
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

	    case RESULT:
		if (write_data.state == W_NONE) {
		    write_data.byte = measurements.temperature;
		    write_data.bit_counter = 8;
		    write_data.success_state = STOP;
		    write_data.error_state = STOP;
		    write_data.state = W_WRITE;
		}
		i2c_write(&write_data, &i2c_state);
		break;

	    case STOP:
		i2c_stop();
		break;
	}
    }

    measurements_state = M_NONE;
}

