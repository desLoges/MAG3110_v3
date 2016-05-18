// Do not remove the include below
#include "MAG3110.h"
#include "i2c.h" // Use I2C library found here: http://users.soe.ucsc.edu/~karplus/Arduino/libraries/i2c/
#include "printf.h"

extern HardwareSerial Serial;

float declination = 75.34 / 1000.0; //for 4.19

mag_HiLo_values_t mag_HiLo_values;
mag_offsets_t mag_offsets;
mag_scales_t mag_scales;

int16_t mag_x, mag_y, mag_z;
float heading;

MAG3110 compass;

void print_heading(int16_t x, int16_t y, int16_t z, mag_HiLo_values_t* HiLo) {

	printf("\n x=%d y=%d z=%d", x, y, z);

	float mag_x_scale = 1.0 / (HiLo->mag_high_x - HiLo->mag_low_x);
	float mag_y_scale = 1.0 / (HiLo->mag_high_y - HiLo->mag_low_y);

	mag_scales.mag_x_scale = 1.0 / (HiLo->mag_high_x - HiLo->mag_low_x);
	mag_scales.mag_y_scale = 1.0 / (HiLo->mag_high_y - HiLo->mag_low_y);
	mag_scales.mag_z_scale = 1.0 / (HiLo->mag_high_z - HiLo->mag_low_z);



	float heading = atan2(-y * mag_y_scale, x * mag_x_scale);



	if (heading < 0) {
		heading += 2 * PI;  // correct for when the heading is negative
	}

	//printf(" heading: %f", heading);

	float headingDegrees = heading * degrees_per_radian;  // convert to degrees

	//printf(" heading: %f", headingDegrees);

}


void setup(void) {
	Serial.begin(115200);
	printf_begin();

	pinMode(13, OUTPUT);

	i2cInit();
	i2cSetBitrate(100);  // try 100kHz

	compass.mag_calibrate(&mag_HiLo_values);

	compass.mag_setup(MAG_3110_SAMPLE1_25, MAG_3110_OVERSAMPLE1);

	compass.mag_set_offsets(&mag_HiLo_values, &mag_offsets); //hard iron calibration

	printf("\nwait serial");
	while (!Serial.available())
	;
}

void loop(void) {

	compass.mag_read_xyz(&mag_x, &mag_y, &mag_z);  // read from sensor
	printf("\n x=%d y=%d z=%d", mag_x, mag_y, mag_z);


	//Apply hard iron calibration
	// mag_x -= (mag_HiLo_values.mag_low_x + mag_HiLo_values.mag_high_x) /2 ;
	// mag_y -= (mag_HiLo_values.mag_low_y + mag_HiLo_values.mag_high_y) /2 ;
	// mag_z -= (mag_HiLo_values.mag_low_z + mag_HiLo_values.mag_high_z) /2 ;

	//Apply soft iron calibration
	mag_scales.mag_x_scale = 1.0 / (mag_HiLo_values.mag_high_x - mag_HiLo_values.mag_low_x);
	mag_scales.mag_y_scale = 1.0 / (mag_HiLo_values.mag_high_y - mag_HiLo_values.mag_low_y);
	mag_scales.mag_z_scale = 1.0 / (mag_HiLo_values.mag_high_z - mag_HiLo_values.mag_low_z);


	// Serial.println();
	// Serial.print("  mag_x_scale=");
	// Serial.print(scales->mag_x_scale * 10000);
	// Serial.print("E-4 mag_y_scale=");
	// Serial.print(scales->mag_y_scale * 10000);
	// Serial.print("E-4 mag_z_scale=");
	// Serial.print(scales->mag_z_scale * 10000);
	// Serial.println("E-4");

	heading = 180 * atan2(mag_y * mag_scales.mag_y_scale, mag_x * mag_scales.mag_x_scale)/M_PI;

	heading += declination * 180/M_PI;

	if(heading < 0)
	heading += 360;

	Serial.print(" heading: ");
	Serial.print(heading);
	Serial.println();

	delay(1000);
}
