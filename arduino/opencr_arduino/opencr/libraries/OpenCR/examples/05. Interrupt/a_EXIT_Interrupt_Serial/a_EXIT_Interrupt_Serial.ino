/*
 * EXIT_Interrupt_Serial
*/

/*
    0: PIN 2,   EXTI_0
    1: PIN 3,   EXTI_1
    2: PIN 4,   EXTI_2
    3: PIN 7,   EXTI_3
    4: PIN 8,   EXTI_4
 */


void setup(){
  Serial.begin(115200);

  pinMode(9, OUTPUT);

  pinMode(2, INPUT_PULLDOWN);
  pinMode(3, INPUT_PULLDOWN);
  pinMode(4, INPUT_PULLDOWN);
  pinMode(7, INPUT_PULLDOWN);
  pinMode(8, INPUT_PULLDOWN);

  /*It can be choose as CHANGE, RISING or FALLING*/
  attachInterrupt(0, changeDirection_EXIT_0, RISING);
  attachInterrupt(1, changeDirection_EXIT_1, RISING);
  attachInterrupt(2, changeDirection_EXIT_2, RISING);
  attachInterrupt(3, changeDirection_EXIT_3, RISING);
  attachInterrupt(4, changeDirection_EXIT_4, RISING);
}

void changeDirection_EXIT_0(void){
  Serial.println("EXIT_Interrupt! 0");
}

void changeDirection_EXIT_1(void){
  Serial.println("EXIT_Interrupt! 1");
}

void changeDirection_EXIT_2(void){
  Serial.println("EXIT_Interrupt! 2");
}

void changeDirection_EXIT_3(void){
  Serial.println("EXIT_Interrupt! 3");
}

void changeDirection_EXIT_4(void){
  Serial.println("EXIT_Interrupt! 4");
}

void loop(){
  digitalWrite(9, HIGH);
  delay(500);
  digitalWrite(9, LOW);
  delay(500);
}
