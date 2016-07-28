/*State change detection
 
 Often, you don't need to know the state of a digital input all the
 time, but you just need to know when the input changes from one state
 to another.  For example, you want to know when a button goes from
 OFF to ON. 
 
                   Compatibility
 CM900                  X
 OpenCM9.04             O

 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

void setup() {
  // initialize the button as an input:
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(BOARD_LED_PIN, OUTPUT);
}

void loop() {
  // read the pushbutton input pin:
  buttonState = digitalRead(BOARD_BUTTON_PIN);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH, then the button went from
      // off to on:
      buttonPushCounter++;
      SerialUSB.println("on");
      SerialUSB.print("number of button pushes:  ");
      SerialUSB.println(buttonPushCounter, DEC);
    }
    else {
      // if the current state is LOW, then the button went from
      // on to off:
      SerialUSB.println("off");
    }

    // save the current state as the last state, for next time
    // through the loop
    lastButtonState = buttonState;
  }

  // turns on the LED every four button pushes by checking the
  // modulo of the button push counter.  Modulo (percent sign, %)
  // gives you the remainder of the division of two numbers:
  if (buttonPushCounter % 4 == 0) {
    digitalWrite(BOARD_LED_PIN, HIGH);
  } 
  else {
    digitalWrite(BOARD_LED_PIN, LOW);
  }
}

