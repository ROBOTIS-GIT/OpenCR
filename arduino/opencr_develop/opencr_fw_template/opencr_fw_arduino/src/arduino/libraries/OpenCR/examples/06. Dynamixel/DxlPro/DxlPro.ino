#include <Dynamixel.h>


Dynamixel Dxl;

int motor_id = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  Dxl.begin(3);

  Dxl.writeByte(motor_id, 64, 1 );
}

void loop() {
  static uint32_t tTime;
  static uint32_t MotorTimer;
  static uint32_t MotorIndex;
  static uint8_t led_tog = 0;

  if( millis()-tTime > 10 )
  {
    tTime = millis();

    Dxl.writeByte(motor_id, 65, led_tog );

    Serial.print(Dxl.readByte(motor_id, 65));
    Serial.print(" ");
    Serial.print(Dxl.readByte(motor_id, 11));
    Serial.print(" ");
    Serial.print(Dxl.readWord(motor_id, 132));
    Serial.println(" ");

    digitalWrite(13, led_tog);
    led_tog ^= 1;
  }


  if( millis()-MotorTimer > 1000 )
  {
    MotorTimer = millis();

    if( MotorIndex == 0 )
    {
      Dxl.writeDword(motor_id, 116, get_pos(0) );
    }
    else
    {
      Dxl.writeDword(motor_id, 116, get_pos(90) );
    }

    MotorIndex ^= 1;
  }

  if( Serial.available() )
  {
    uint8_t ch;

    ch = Serial.read();

    if( ch == '1' ) Dxl.writeByte(motor_id, 64, 1 );
    if( ch == '2' ) Dxl.writeByte(motor_id, 64, 0 );
    if( ch == '3' ) Dxl.writeDword(motor_id, 116, get_pos(0) );
    if( ch == '4' ) Dxl.writeDword(motor_id, 116, get_pos(90) );

  }
}

uint32_t get_pos( int pos )
{
  return map( pos, -180, 180, 0, 4095);
}

