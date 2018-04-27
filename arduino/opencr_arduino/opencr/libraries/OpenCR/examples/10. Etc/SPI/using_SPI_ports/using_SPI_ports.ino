/*
 * SPI Examble
 */



#include <SPI.h>


#define SPI_NSS_PIN  10



void setup() {

  SPI.begin();                          // Initialize the SPI port.
  SPI.setBitOrder(MSBFIRST);            // Set the SPI bit order
  SPI.setDataMode(SPI_MODE0);           //Set the  SPI data mode 0
  SPI.setClockDivider(SPI_CLOCK_DIV16); // Slow speed (108 / 16 = 6.75 MHz SPI speed)
  pinMode(SPI_NSS_PIN, OUTPUT);
}

void loop() {

  sendSPI();

  delayMicroseconds(10);
}

void sendSPI()
{
  digitalWrite(SPI_NSS_PIN, LOW);
  SPI.transfer(0x55);
  digitalWrite(SPI_NSS_PIN, HIGH);
}
