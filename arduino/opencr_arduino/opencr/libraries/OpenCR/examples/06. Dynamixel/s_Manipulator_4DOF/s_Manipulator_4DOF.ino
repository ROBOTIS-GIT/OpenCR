/* Manipulator 4DOF
 
                Compatibility
 CM900                  X
 OpenCM9.04             O
 
                  Dynamixel Compatibility
               AX    MX      RX    XL-320    Pro
 CM900          O      X      X        X      X
 OpenCM9.04     O      X      X        X      X
 **** OpenCM 485 EXP board is needed to use 4 pin Dynamixel and Pro Series ****  
 
 created 22 May 2014
 by ROBOTIS CO,.LTD.
 */
 #include <OLLO.h>
OLLO myOLLO;
 
// My gigantic dynamixel header file
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP


Dynamixel Dxl(DXL_BUS_SERIAL1);


#define NUM_ACTUATOR        4
word  GoalPos[NUM_ACTUATOR], PrevGoalPos[NUM_ACTUATOR];
byte  id[NUM_ACTUATOR];
word wGoalPos[NUM_ACTUATOR];

int SyncPage1[8]=
{ 
  3,0,  
  4,0,
  5,0,
  6,0}; 

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);

  int i = 0;
  myOLLO.begin(1, IR_SENSOR);
  myOLLO.begin(2);
  myOLLO.begin(3);
  Dxl.writeWord(3, 30, 512);
  Dxl.writeWord(4, 30, 512);
  Dxl.writeWord(5, 30, 512);
  Dxl.writeWord(6, 30, 512);
  myOLLO.write(2,1,1);
  myOLLO.write(3,1,1);
  wGoalPos[0]=	512	;
  wGoalPos[1]=	512	;
  wGoalPos[2]=	512	;
  wGoalPos[3]=	512	;
  PrevGoalPos[0]=	512	;
  PrevGoalPos[1]=	512	;
  PrevGoalPos[2]=	512	;
  PrevGoalPos[3]=	512	;
  delay(3000);

  for(i = 3;i<=6;i++){
    Dxl.writeByte(i, 28, 128);
    Dxl.writeByte(i, 29, 128);
  }

  motionpage_1();
  motionpage_2();
}


void loop(){

  motionpage_3();
  sensor_stop();
  motionpage_4();
  sensor_stop();
  motionpage_5();
  sensor_stop();
  motionpage_6();
  sensor_stop();
  motionpage_7();
  sensor_stop();
  motionpage_8();
  sensor_stop();
  
}


void MotionPagePlay(word * wGoalPos, word wTimeMill, word wPauseTime)
{
  delay(wPauseTime);
  word wNumOfStep = wTimeMill / 8;
  word wCount;
  int i = 0;
  int ii = 1;
  word wTempGoalPos[NUM_ACTUATOR];
  for (wCount = 0; wCount < wNumOfStep; wCount++)
  {
    for (i = 0; i<NUM_ACTUATOR; i++)
    {
      if(wGoalPos[i] > PrevGoalPos[i])
        GoalPos[i] = PrevGoalPos[i] + (wGoalPos[i] - PrevGoalPos[i]) * wCount / wNumOfStep;
      else
        GoalPos[i] = PrevGoalPos[i] - (PrevGoalPos[i] - wGoalPos[i]) * wCount / wNumOfStep;
    }
    i=0;
    for(ii = 1;ii<=9;ii+=2){
      SyncPage1[ii] = GoalPos[i];
      i++;
    }

    Dxl.syncWrite(30,1, SyncPage1, 8);
    delay(8);

  }
  for (i = 0; i<NUM_ACTUATOR; i++)
  {
    PrevGoalPos[i] = wGoalPos[i];
  }
  delay(100);
}


void motionpage_1(){
  wGoalPos[0]=	512	;
  wGoalPos[1]=	512	;
  wGoalPos[2]=	512	;
  wGoalPos[3]=	512	;
  MotionPagePlay(wGoalPos, 700, 200);
}
void motionpage_2(){
  wGoalPos[0]=	512	;
  wGoalPos[1]=	512	;
  wGoalPos[2]=	400	;
  wGoalPos[3]=	450	;
  MotionPagePlay(wGoalPos, 700, 200);
} 
void motionpage_3(){
  wGoalPos[0]=	380	;
  wGoalPos[1]=	400	;
  wGoalPos[2]=	400	;
  wGoalPos[3]=	450	;
  MotionPagePlay(wGoalPos, 700, 200);
} 
void motionpage_4(){
  wGoalPos[0]=	400	;
  wGoalPos[1]=	400	;
  wGoalPos[2]=	520	;
  wGoalPos[3]=	500	;
  MotionPagePlay(wGoalPos, 700, 200);
} 
void motionpage_5(){
  wGoalPos[0]=	400	;
  wGoalPos[1]=	400	;
  wGoalPos[2]=	580	;
  wGoalPos[3]=	650	;
  MotionPagePlay(wGoalPos, 700, 200);
} 
void motionpage_6(){
  wGoalPos[0]=	670	;
  wGoalPos[1]=	680	;
  wGoalPos[2]=	570	;
  wGoalPos[3]=	610	;
  MotionPagePlay(wGoalPos, 700, 200);
} 
void motionpage_7(){
  wGoalPos[0]=	680	;
  wGoalPos[1]=	680	;
  wGoalPos[2]=	530	;
  wGoalPos[3]=	510	;
  MotionPagePlay(wGoalPos, 700, 200);
} 
void motionpage_8(){
  wGoalPos[0]=	670	;
  wGoalPos[1]=	680	;
  wGoalPos[2]=	420	;
  wGoalPos[3]=	450	;
  MotionPagePlay(wGoalPos, 700, 200);
} 

void sensor_stop(){
  if(myOLLO.read(1, IR_SENSOR) > 500){
    motionpage_1();
    myOLLO.write(2,0,0);
    myOLLO.write(3,0,0);
    delay(4000);
  }
  else{
    myOLLO.write(2,1,1);
    myOLLO.write(3,1,1);
  }

}










