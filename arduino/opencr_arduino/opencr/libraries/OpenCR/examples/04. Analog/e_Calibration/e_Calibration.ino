/* Calibration
 
 Demonstrates one techinque for calibrating sensor input.  The sensor
 readings during the first five seconds of the sketch execution
 define the minimum and maximum of expected values attached to the
 sensor pin.
 
 The sensor minumum and maximum initial values may seem backwards.
 Initially, you set the minimum high and listen for anything lower,
 saving it as the new minumum.  Likewise, you set the maximum low and
 listen for anything higher as the new maximum.
 
 The circuit:
 Analog sensor (potentiometer will do) attached to analog input 1
 
                   Compatibility
 CM900                  O
 OpenCM9.04             O

 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */

// Constant (won't change):
const int sensorPin = 2;    // pin that the sensor is attached to

// Variables:
int sensorMin = 0;   // minimum sensor value
int sensorMax = 4095;      // maximum sensor value
int sensorValue = 0;    // the sensor value

void setup() {
  // Declare the sensorPin as INPUT_ANALOG:
  pinMode(sensorPin, INPUT_ANALOG);

  // Turn on the built-in LED to signal the start of the calibration
  // period:
  pinMode(BOARD_LED_PIN, PWM);
  digitalWrite(BOARD_LED_PIN, HIGH);

  // Calibrate during the first five seconds:
  while (millis() < 5000) {
    sensorValue = analogRead(sensorPin);

    // Record the maximum sensor value:
    if (sensorValue > sensorMax) {
      sensorMax = sensorValue;
    }

    // Record the minimum sensor value:
    if (sensorValue < sensorMin) {
      sensorMin = sensorValue;
    }
  }

  // Signal the end of the calibration period:
  digitalWrite(BOARD_LED_PIN, LOW);
}

void loop() {
  // Read the sensor:
  sensorValue = analogRead(sensorPin);

  // Apply the calibration to the sensor reading:
  sensorValue = map(sensorValue, sensorMin, sensorMax, 0, 65535);

  // In case the sensor value is outside the range seen during calibration:
  sensorValue = constrain(sensorValue, 0, 65535);

  // Fade the LED using the calibrated value:
  analogWrite(BOARD_LED_PIN, sensorValue);
  delay(100);
}

