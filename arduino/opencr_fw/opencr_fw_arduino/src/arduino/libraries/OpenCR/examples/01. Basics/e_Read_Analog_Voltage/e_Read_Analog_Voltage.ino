/*
 * ReadAnalogVoltage
 */

void setup() {
  Serial.begin(115200);
}

void loop() {
  int adc_value;
  float vol_value;

  // put your main code here, to run repeatedly:

  adc_value = analogRead(BDPIN_BAT_PWR_ADC);

  Serial.print("BAT_PWR = ");
  Serial.print(adc_value);
  Serial.print("\t");
  vol_value = map(adc_value, 0, 1023, 0, 330*57/10);
  vol_value = vol_value/100;
  Serial.print(vol_value);
  Serial.println("V\t");

  delay(100);
}
