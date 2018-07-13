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

/* Authors: Hye-Jong KIM */


#include "OMManager.h"
  Joint omjoint[3];
  Link omlink[3];
  Tool omtool[1];
  float a = 0.0;
void setup()
{
  Serial.begin(57600);

  omjoint[0].Init("joint1", 1, 1);
  omjoint[1].Init("joint2", 2, 2);
  omjoint[2].Init("joint2", 3, 0);
  

  omlink[0].Init("link1",2);
  omlink[1].Init("link2",3);


  //omtool.Init("Suction", 1, (0,1,0), (1,0,0,0,1,0,0,0,1))

  Manipulator om("oml", 3);
  om.Load(omjoint, omlink, omtool);

}

void loop()
{
  float a = a + 1.0;
  omjoint[1].SetAngle(a);
  Serial.print(omjoint[1].GetAngle());
  delay(100);
}



