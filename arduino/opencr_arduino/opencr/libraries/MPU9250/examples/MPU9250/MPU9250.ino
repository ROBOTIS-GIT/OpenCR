/**
 * Sample program for the MPU9250 using SPI
 *
 * Sample rate of the AK8963 magnetometer is set at 100Hz. 
 * There are only two options: 8Hz or 100Hz so I've set it at 100Hz
 * in the library. This is set by writing to the CNTL1 register
 * during initialisation.
 *
 * Copyright (C) 2015 Brian Chen
 *
 * Open source under the MIT license. See LICENSE.txt.
 */

#include <SPI.h>
#include <MPU9250.h>

#define SPI_CLOCK 8000000  // 8MHz clock works.

#define SS_PIN   10 
#define INT_PIN  3
#define LED      13

#define WAITFORINPUT(){            \
	while(!Serial.available()){};  \
	while(Serial.available()){     \
		Serial.read();             \
	};                             \
}                                  \

MPU9250 mpu(SPI_CLOCK, SS_PIN);

void setup() {
	Serial.begin(115200);

	pinMode(INT_PIN, INPUT);
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);

	SPI.begin();

	Serial.println("Press any key to continue");
	WAITFORINPUT();

	mpu.init(true);

	uint8_t wai = mpu.whoami();
	if (wai == 0x71){
		Serial.println("Successful connection");
	}
	else{
		Serial.print("Failed connection: ");
		Serial.println(wai, HEX);
	}

	uint8_t wai_AK8963 = mpu.AK8963_whoami();
	if (wai_AK8963 == 0x48){
		Serial.println("Successful connection to mag");
	}
	else{
		Serial.print("Failed connection to mag: ");
		Serial.println(wai_AK8963, HEX);
	}

	mpu.calib_acc();
	mpu.calib_mag();

	Serial.println("Send any char to begin main loop.");
	WAITFORINPUT();
}

void loop() {
	// various functions for reading
	// mpu.read_mag();
	// mpu.read_acc();
	// mpu.read_gyro();

	mpu.read_all();

	Serial.print(mpu.gyro_data[0]);   Serial.print('\t');
	Serial.print(mpu.gyro_data[1]);   Serial.print('\t');
	Serial.print(mpu.gyro_data[2]);   Serial.print('\t');
	Serial.print(mpu.accel_data[0]);  Serial.print('\t');
	Serial.print(mpu.accel_data[1]);  Serial.print('\t');
	Serial.print(mpu.accel_data[2]);  Serial.print('\t');
	Serial.print(mpu.mag_data[0]);    Serial.print('\t');
	Serial.print(mpu.mag_data[1]);    Serial.print('\t');
	Serial.print(mpu.mag_data[2]);    Serial.print('\t');
	Serial.println(mpu.temperature);

	delay(10);
}
