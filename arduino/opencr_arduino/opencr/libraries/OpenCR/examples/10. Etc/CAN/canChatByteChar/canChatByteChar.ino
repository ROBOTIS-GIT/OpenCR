uint32_t id;
uint8_t data;

void setup() 
{
  Serial.begin(115200);
  
  if(canOpen(_DEF_CAN_BAUD_125K, _DEF_CAN_EXT) == false)
  {
    Serial.println("Can open fail!!");
  }
  else
  {
    id = 0x123;
    canConfigFilter(0, 0);  
  }
}

void loop() 
{
  if(canAvailable())
  {
    Serial.print((char)canRead());
  }

  if(Serial.available())
  {
    data = Serial.read();
    canWrite(id, &data, 1);
  }
}
