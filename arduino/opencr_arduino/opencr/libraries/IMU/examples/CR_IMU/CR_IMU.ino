#include <IMU.h>


cIMU    IMU;



uint8_t   err_code;
uint8_t   led_tog = 0;
uint8_t   led_pin = 13;

void setup() 
{
  Serial.begin(115200);

  pinMode( led_pin, OUTPUT );
  err_code = IMU.begin();
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
    Serial.print(" ");
    Serial.print(IMU.angle[0]/10);
    Serial.print(" ");
    Serial.print(IMU.angle[1]/10);
    Serial.print(" ");
    Serial.println(IMU.angle[2]);
  }


  if( Serial.available() )
  {
    char Ch = Serial.read();

    if( Ch == '1' )
    {
      Serial.println("ACC Cali Start");

      IMU.SEN.acc_cali_start();
      while( IMU.SEN.acc_cali_get_done() == false )
      {
        IMU.update();
        //Serial.println( IMU.SEN.calibratingA );
      }

      Serial.print("ACC Cali End ");
    }
  }
}

