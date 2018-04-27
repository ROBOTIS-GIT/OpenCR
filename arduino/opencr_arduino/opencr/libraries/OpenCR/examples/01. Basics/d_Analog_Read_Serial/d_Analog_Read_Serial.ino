/*
 * AnalogReadSerial
 */



void setup(){
  Serial.begin(115200);
}

void loop(){
  Serial.print("A0 = ");
  Serial.print(analogRead(A0));
  Serial.print("\t");

  Serial.print("A1 = ");
  Serial.print(analogRead(A1));
  Serial.print("\t");

  Serial.print("A2 = ");
  Serial.print(analogRead(A2));
  Serial.print("\t");

  Serial.print("A3 = ");
  Serial.print(analogRead(A3));
  Serial.print("\t");

  Serial.print("A4 = ");
  Serial.print(analogRead(A4));
  Serial.print("\t");

  Serial.print("A5 = ");
  Serial.print(analogRead(A5));
  Serial.println(" ");

  delay(100);
}
