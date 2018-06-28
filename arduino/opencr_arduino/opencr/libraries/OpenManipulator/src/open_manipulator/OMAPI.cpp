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

/* Authors: Hye-Jong KIM*/



#include "../../include/open_manipulator/OMAPI.h"
#include "../../include/open_manipulator/OMManager.h"



void ManipulatorInit(Manipulator* manipulator, int8_t manipulator_number, String manipulator_name, int8_t dof, int8_t number_of_joint, int8_t number_of_link, int8_t number_of_tool)
{
  manipulator[manipulator_number].Init(manipulator_name, dof, number_of_joint, number_of_link, number_of_tool);
 
  

}

/*
Manipulator OM;
OM.Init("OpenManipulatorLink",3,7,5,1);//name_, dof,number of joint, number of link, number of tool

OM.joint[0].Init("YawBase", 1, 1);
OM.joint[1].Init("PitchJoint1", 2, 2);
OM.joint[2].Init("PitchJoint2", 3, 3);
OM.joint[3].Init("PissiveJoint1", 4, -1);
OM.joint[4].Init("PissiveJoint2", 5, -1);
OM.joint[5].Init("PissiveJoint3", 6, -1);
OM.joint[6].Init("PissiveJoint4", 7, -1);

OM.link[0].Init("link1",3);
OM.link[0].joint[0].Init(1);
OM.link[0].joint[1].Init(2);
OM.link[0].joint[2].Init(3);

OM.link[1].Init("link2",2);
OM.link[2].Init("link3",2);
OM.link[3].Init("link4",2);
OM.link[4].Init("link5",2);

OM.tool[0].Init("link5",2);
*/
