#include "MAG3110.h"
#include "i2c.h"

#define mag_write_reg(r,v)   (i2cWriteRegister(MAG_3110_I2C,r,v))
#define mag_read_reg(r)      (i2cReadRegister(MAG_3110_I2C,r))

inline uint8_t mag_data_ready(void) {
	return mag_read_reg(MAG_3110_DR_STATUS) & MAG_3110_ZYXDR;
}



void MAG3110::mag_set_offsets(mag_HiLo_values_t* HiLo, mag_offsets_t* offsets) {

	offsets->mag_x_offset = (HiLo->mag_low_x + HiLo->mag_high_x) / 2;
	offsets->mag_y_offset = (HiLo->mag_low_y + HiLo->mag_high_y) / 2;
	offsets->mag_z_offset = (HiLo->mag_low_z + HiLo->mag_high_z) / 2;

	printf("\nx_off=%d",offsets->mag_x_offset);
	printf(" y_off=%d",offsets->mag_y_offset);
	printf(" z_off=%d",offsets->mag_z_offset);

	static uint8_t data[6];
	data[0] = offsets->mag_x_offset >> 7;
	data[1] = (offsets->mag_x_offset << 1) & 0xFF;
	data[2] = offsets->mag_y_offset >> 7;
	data[3] = (offsets->mag_y_offset << 1) & 0xFF;
	data[4] = offsets->mag_z_offset >> 7;
	data[5] = (offsets->mag_z_offset << 1) & 0xFF;

	i2cWriteRegisters(MAG_3110_I2C, MAG_3110_OFF_X_MSB, 6, data);
}

//public functions
void MAG3110::mag_read_xyz(int16_t *x, int16_t *y, int16_t *z) {
	while (!mag_data_ready()) {
	} // wait for new set of data

	static uint8_t data[6];
	i2cReadRegisters(MAG_3110_I2C, MAG_3110_OUT_X_MSB, 6, data);
	*x = (data[0] << 8) + data[1];
	*y = (data[2] << 8) + data[3];
	*z = (data[4] << 8) + data[5];
}

void MAG3110::mag_setup(uint8_t samplerate, uint8_t oversamle) {


	mag_write_reg(MAG_3110_CTRL_REG2, MAG_3110_AUTO_MRST_EN);

	mag_write_reg(MAG_3110_CTRL_REG1,
		samplerate+oversamle+MAG_3110_ACTIVE);
	}

	void MAG3110::print_registers(void) {
		Serial.println("MAG3110 registers:");

		for (int i = 0; i < 0x12; i++) {
			Serial.print(i, HEX);
			Serial.print(": 0x");
			Serial.println(mag_read_reg(i), HEX);
			delay(2);
		}
	}


	void MAG3110::mag_calibrate(mag_HiLo_values_t* HiLo) {

		//uint8_t mag_calibrate = 1;	  // turn on during calibration
		uint16_t mag_num_calib = 0;  // number of readings when calibrating
		bool toggle = false;

		printf("\nCalibration");

		mag_write_reg(MAG_3110_CTRL_REG2, MAG_3110_AUTO_MRST_EN);

		mag_write_reg(MAG_3110_CTRL_REG1,
			MAG_3110_SAMPLE80+MAG_3110_OVERSAMPLE1+MAG_3110_ACTIVE);

		mag_write_reg(MAG_3110_CTRL_REG2,	mag_read_reg(MAG_3110_CTRL_REG2)| MAG_3110_RAW);

		printf("\nREG1 %d", mag_read_reg(MAG_3110_CTRL_REG1));

		HiLo->mag_low_x = 32767;
		HiLo->mag_high_x = 0x8000;
		HiLo->mag_low_y = 32767;
		HiLo->mag_high_y = 0x8000;
		HiLo->mag_low_z = 32767;
		HiLo->mag_high_z = 0x8000;
		// scales->mag_x_scale = 1;
		// scales->mag_y_scale = 1;
		// scales->mag_z_scale = 1;
		mag_num_calib = 0;

		int16_t x, y, z;
		mag_read_xyz(&x, &y, &z);  // discard a read from magnetometer (not RAW)

		while (mag_num_calib < MAG_CALIB_NUM_LIMIT) {

			if (mag_data_ready()) {

				mag_read_xyz(&x, &y, &z);

				if (x < HiLo->mag_low_x)
				HiLo->mag_low_x = x;
				if (x > HiLo->mag_high_x)
				HiLo->mag_high_x = x;
				if (y < HiLo->mag_low_y)
				HiLo->mag_low_y = y;
				if (y > HiLo->mag_high_y)
				HiLo->mag_high_y = y;
				if (z < HiLo->mag_low_z)
				HiLo->mag_low_z = z;
				if (z > HiLo->mag_high_z)
				HiLo->mag_high_z = z;

				mag_num_calib++;

				digitalWrite(13, toggle);
				toggle=!toggle;
			}
		}

		mag_num_calib = 0;
		digitalWrite(13, LOW);

		printf("\nmag_low_x=%d",HiLo->mag_low_x);
		printf(" mag_high_x=%d",HiLo->mag_high_x);
		printf("\nmag_low_y=%d",HiLo->mag_low_y);
		printf(" mag_high_y=%d",HiLo->mag_high_y);
		printf("\nmag_low_z=%d",HiLo->mag_low_z);
		printf(" mag_high_z=%d",HiLo->mag_high_z);


		// stop using raw mode
		mag_write_reg(MAG_3110_CTRL_REG2,
			mag_read_reg(MAG_3110_CTRL_REG2) &~ MAG_3110_RAW);

			mag_read_xyz(&x, &y, &z);  // discard a read from magnetometer (was RAW)

			printf("\nCalibrated");
		}
