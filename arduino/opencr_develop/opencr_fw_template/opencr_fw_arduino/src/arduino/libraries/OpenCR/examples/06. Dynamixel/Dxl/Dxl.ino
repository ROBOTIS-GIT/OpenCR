#include <Dynamixel.h>


Dynamixel Dxl;

int motor_id = 1;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  Dxl.begin(3);
  Dxl.writeByte(motor_id, 24, 1 );
}

void loop() {
  static uint32_t tTime;
  static uint32_t MotorTimer;
  static uint32_t MotorIndex;
  static uint8_t led_tog = 0;

  if( millis()-tTime > 10 )
  {
    tTime = millis();


    Serial.print(Dxl.readWord(motor_id, 36));
    Serial.println(" ");

    digitalWrite(13, led_tog);
    led_tog ^= 1;
  }


  if( millis()-MotorTimer > 1000 )
  {
    MotorTimer = millis();

    if( MotorIndex == 0 )
    {
      Dxl.writeByte(motor_id, 25, 0 );
      Dxl.writeWord(motor_id, 30, get_pos(0) );
    }
    else
    {
      Dxl.writeByte(motor_id, 25, 1 );
      Dxl.writeWord(motor_id, 30, get_pos(90) );
    }

    MotorIndex ^= 1;
  }

  if( Serial.available() )
  {
    uint8_t ch;

    ch = Serial.read();

    if( ch == '1' ) Dxl.writeByte(motor_id, 24, 1 );
    if( ch == '2' ) Dxl.writeByte(motor_id, 24, 0 );
  }
}

uint32_t get_pos( int pos )
{
  return map( pos, -180, 180, 0, 1023);
}

