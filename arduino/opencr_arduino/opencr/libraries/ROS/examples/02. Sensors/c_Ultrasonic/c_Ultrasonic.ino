/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho, Gilbert */
uint32_t t_time;
uint32_t pre_time;
uint32_t start_time;
uint32_t end_time;
uint32_t count_start = 0;
uint32_t data = 1;

float duration;

const int echoPin = BDPIN_GPIO_1;
const int trigPin = BDPIN_GPIO_2;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
    digitalWrite(trigPin, data);
    if (millis()-pre_time >= 10 && data == 1)
    {
      data = 0;
      pre_time = millis();
    }
    
    if (millis()-pre_time >= 5 && data == 0)
    {
      data = 1;
      pre_time = millis();
    }

    if (digitalRead(echoPin) == HIGH && count_start == 0)
    {
      start_time = micros();
      count_start = 1;
    }

    else if (digitalRead(echoPin) == LOW && count_start == 1)
    {
      end_time = micros();
      count_start = 0;
      duration = (end_time - start_time) / 2 / 29.1;
      Serial.print("t_time: ");
      Serial.println(duration);
    }
}
