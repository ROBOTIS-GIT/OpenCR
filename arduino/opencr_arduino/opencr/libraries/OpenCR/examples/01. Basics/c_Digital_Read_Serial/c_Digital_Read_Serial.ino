/*
 * DigitalReadSerial
 */

/*
#define BDPIN_DIP_SW_1          26
#define BDPIN_DIP_SW_2          27
#define BDPIN_PUSH_SW_1         34
#define BDPIN_PUSH_SW_2         35
 */

void setup(){
  Serial.begin(115200);

  pinMode(BDPIN_DIP_SW_1, INPUT);
  pinMode(BDPIN_DIP_SW_2, INPUT);
  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);

}
void loop(){
  int dip_state;
  int push_state;

  dip_state  = digitalRead(BDPIN_DIP_SW_1)<<0;
  dip_state |= digitalRead(BDPIN_DIP_SW_2)<<1;

  push_state  = digitalRead(BDPIN_PUSH_SW_1)<<0;
  push_state |= digitalRead(BDPIN_PUSH_SW_2)<<1;

  Serial.print("dip_state = ");
  Serial.print(dip_state, BIN);

  Serial.print("\tpush_state = ");
  Serial.println(push_state, BIN);


  delay(100);
}


