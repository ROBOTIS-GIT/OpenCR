/*
  Copyright (c) 2013 Arduino LLC. All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <Arduino.h>

#include "Servo.h"
#include "drv_pwm.h"



#define SERVO_DEFAULT_FREQ          (1000000/REFRESH_INTERVAL)

// Unit conversions
#define ANGLE_TO_US(a)    ((uint16_t)(map((a), this->minAngle, this->maxAngle, \
                                        this->minPW, this->maxPW)))
#define US_TO_ANGLE(us)   ((int16_t)(map((us), this->minPW, this->maxPW,  \
                                       this->minAngle, this->maxAngle)))

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)  // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)  // maximum value in uS for this servo


Servo::Servo()
{
    this->resetFields();
}

uint8_t Servo::attach(int pin)
{
  return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t Servo::attach(int pin, int min, int max)
{
    //analogWriteResolution(16);
    this->pin = pin;
    this->min = (MIN_PULSE_WIDTH - min)/4;
    this->max = (MIN_PULSE_WIDTH - min)/4;

    drv_pwm_set_freq((uint32_t)pin, SERVO_DEFAULT_FREQ);
    drv_pwm_setup((uint32_t)pin);

    this->is_attached = true;

    return true;
}

bool Servo::attached()
{
    return this->is_attached;
}

void Servo::detach()
{
    drv_pwm_release(this->pin);
}

void Servo::write(int value)
{
  if(value < MIN_PULSE_WIDTH)
  {  // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
    if(value < 0) value = 0;
    if(value > 180) value = 180;
    value = map(value, 0, 180, SERVO_MIN(),  SERVO_MAX());
  }
  this->writeMicroseconds(value);
}

int Servo::read() {
    int a = US_TO_ANGLE(this->readMicroseconds());
    // map() round-trips in a weird way we mostly correct for here;
    // the round-trip is still sometimes off-by-one for write(1) and
    // write(179).
    return a == this->minAngle || a == this->maxAngle ? a : a + 1;
}

void Servo::writeMicroseconds(int pulseWidth) {
    if (!this->attached()) {
        return;
    }

    pulseWidth = constrain(pulseWidth, this->minPW, this->maxPW);
    pulseWidth = map(pulseWidth, 0, REFRESH_INTERVAL, 0, 4096);

    drv_pwm_set_duty(this->pin, 12, pulseWidth);
}

int Servo::readMicroseconds() {
    if (!this->attached()) {
        return 0;
    }
    
    uint32_t ret_us;
    uint32_t pwm_period = drv_pwm_get_period(this->pin);
    uint32_t pwm_pulse = drv_pwm_get_pulse(this->pin);
    ret_us = map(pwm_pulse, 0, pwm_period, 0, REFRESH_INTERVAL);

    return (int) ret_us;
}

void Servo::resetFields(void)
{
    this->is_attached = false;
    this->minAngle = MIN_ANGLE;
    this->maxAngle = MAX_ANGLE;
    this->minPW = MIN_PULSE_WIDTH;
    this->maxPW = MAX_PULSE_WIDTH;
}