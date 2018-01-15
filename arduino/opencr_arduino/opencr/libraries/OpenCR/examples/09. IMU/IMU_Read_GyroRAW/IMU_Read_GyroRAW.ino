/*
  Range   : +/- 2000 deg/sec
  Scale   : 16.4 = 1 deg/sec
 */

#include <IMU.h>


cIMU    IMU;


uint8_t   led_tog = 0;
uint8_t   led_pin = 13;

void setup()
{
  Serial.begin(115200);

  IMU.begin();

  pinMode( led_pin, OUTPUT );
}





void loop()
{
  static uint32_t tTime[3];
  static uint32_t imu_time = 0;


  if( (millis()-tTime[0]) >= 500 )
  {
    tTime[0] = millis();

    digitalWrite( led_pin, led_tog );
    led_tog ^= 1;
  }

  tTime[2] = micros();
  if( IMU.update() > 0 ) imu_time = micros()-tTime[2];



  if( (millis()-tTime[1]) >= 50 )
  {
    tTime[1] = millis();

    Serial.print(imu_time);
    Serial.print(" \t");
    Serial.print(IMU.gyroRaw[0]);    // GYRO X
    Serial.print(" \t");
    Serial.print(IMU.gyroRaw[1]);    // GYRO Y
    Serial.print(" \t");
    Serial.print(IMU.gyroRaw[2]);    // GYRO Z
    Serial.println(" ");
  }
}
