#include <Dynamixel.h>


Dynamixel Dxl;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  Dxl.begin(3);

  Dxl.writeByte(1, 64, 1 );
}

void loop() {
  static uint32_t tTime;
  static uint32_t MotorTimer;
  static uint32_t MotorIndex;
  static uint8_t led_tog = 0;

  if( millis()-tTime > 10 )
  {
    tTime = millis();

    Dxl.writeByte(1, 65, led_tog );

    Serial.print(Dxl.readByte(1, 65));
    Serial.print(" ");
    Serial.print(Dxl.readByte(1, 11));
    Serial.print(" ");
    Serial.print(Dxl.readWord(1, 132));
    Serial.println(" ");

    digitalWrite(13, led_tog);
    led_tog ^= 1;
  }


  if( millis()-MotorTimer > 1000 )
  {
    MotorTimer = millis();

    if( MotorIndex == 0 )
    {
      Dxl.writeDword(1, 116, get_pos(0) );
    }
    else
    {
      Dxl.writeDword(1, 116, get_pos(90) );
    }

    MotorIndex ^= 1;
  }

  if( Serial.available() )
  {
    uint8_t ch;

    ch = Serial.read();

    if( ch == '1' ) Dxl.writeByte(1, 64, 1 );
    if( ch == '2' ) Dxl.writeByte(1, 64, 0 );
    if( ch == '3' ) Dxl.writeDword(1, 116, get_pos(0) );
    if( ch == '4' ) Dxl.writeDword(1, 116, get_pos(90) );

  }
}

uint32_t get_pos( int pos )
{
  return map( pos, -180, 180, 0, 4095);
}

