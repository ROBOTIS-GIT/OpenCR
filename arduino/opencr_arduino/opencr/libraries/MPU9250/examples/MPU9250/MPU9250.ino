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

#define SS_PIN   28
#define INT_PIN  3
#define LED      9

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


  Serial.println("Send any char to begin main loop.");
  WAITFORINPUT();
}

void loop() {
  // various functions for reading
  mpu.read_acc();
  mpu.read_gyro();

  Serial.print(mpu.gyro_data[0]);   Serial.print('\t');
  Serial.print(mpu.gyro_data[1]);   Serial.print('\t');
  Serial.print(mpu.gyro_data[2]);   Serial.print('\t');
  Serial.print(mpu.accel_data[0]*100);  Serial.print('\t');
  Serial.print(mpu.accel_data[1]*100);  Serial.print('\t');
  Serial.print(mpu.accel_data[2]*100);  Serial.print('\t');
  Serial.println(mpu.temperature);

  delay(10);
}
