#include "I2Cdev.h"
#include "MPU6050.h"


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69


MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high


int16_t ax, ay, az;
int16_t gx, gy, gz;


#define LED_PIN 13
bool blinkState = false;

void setup() {

  I2Cdev::begin(400);


  Serial.begin(115200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);

  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g:\t");
  Serial.print(ax/100); Serial.print("\t");
  Serial.print(ay/100); Serial.print("\t");
  Serial.print(az/100); Serial.print("\t");
  Serial.print(gx/100); Serial.print("\t");
  Serial.print(gy/100); Serial.print("\t");
  Serial.println(gz/100);

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}
