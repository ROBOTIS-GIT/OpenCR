
/* Forward kinematics for 4DOF
  

                Compatibility
 CM900                  O
 OpenCM9.04             O
 
                 Dynamixel Compatibility
               Ax    MX      Rx    XL-320    Pro
 CM900          O      X      X        X      X
 OpenCM9.04     O      X      X        X      X
 **** OpenCM9.04 MX-Series and Pro-Series in order to drive should be OpenCM 485EXP board ****
 
 created 22 May 2014
 by ROBOTIS CO,.LTD.
*/
 /* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL1);

float D2R = 3.14f/180.0f;//dgree -> radian
float l1 = 0.02;	//length of link as mm
float l2 = 0.1;  		
float l3 = 0.13;  		
float d1 = 0.07;   	

float q1 = -0*D2R;	// θ1=q1, θ2=q2, θ3=q3
float q2 = 0*D2R;	//
float q3 = 90*D2R;	//

float x = 0.0f;		// variable for θ1, θ2, θ3
float y = 0.0f;
float z = 0.0f;
//Dxl IDs for 4DOF
int JOINT1_ID = 6;	
int JOINT2_ID = 5;	
int JOINT3_ID = 4;
int  HAND_ID = 3;

void setup() 
{
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  
}
//codes implemented forward kinematics
void forward_kinematics()
{
  float s1 = sin(q1);
  float c1 = cos(q1);
  float s2 = sin(q2);
  float c2 = cos(q2);
  float s3 = sin(q3);
  float c3 = cos(q3);
  //x, y, z for the expression
  x = l1*c1 + l2*c1*c2 + l3*(c1*c2*c3-s1*s3);	
  y = l1*s1 + l2*s1*c2 + l3*(s1*c2*c3+c1*s3);
  z = d1 - l2*s2 - l3*s2*c3;
}
int deg2pos(float de)
{
  int ret;
  // 512 is the center point represents a center of 0 degrees
  //Angle from side to side relative to the center point is calculated.
  //Value of the input angle in order to match the angle of the motor was multiplied by 197.
  ret = de *  197;
  ret = ret+512;
  return ret;	
}
void motion_update()
{
  //move to calculated position using address 30
  Dxl.writeWord(JOINT1_ID, 30, deg2pos(q1));  
  Dxl.writeWord(JOINT2_ID, 30, deg2pos(q2));  
  Dxl.writeWord(JOINT3_ID, 30, deg2pos(q3));	
}
//Display input angle and position
void motion_print()	
{
  SerialUSB.print("angles:");    
  SerialUSB.print("  ");    
  SerialUSB.print(q1);
  SerialUSB.print("  ");
  SerialUSB.print(q2);
  SerialUSB.print("  ");
  SerialUSB.print(q3);
  SerialUSB.println("  ");

  SerialUSB.print("end-effector pos.:");    
  SerialUSB.print("  ");    
  SerialUSB.print(x);
  SerialUSB.print("  ");
  SerialUSB.print(y);
  SerialUSB.print("  ");
  SerialUSB.print(z);
  SerialUSB.println("  ");
}

void loop() 
{
  //moves 4 DOF manipulator
  forward_kinematics();  	
  motion_update();
  motion_print();
}


