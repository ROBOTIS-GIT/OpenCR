/* Smart FlagRobot Example
 
 This example shows R+SMART Kit FlagRobot example
 You can find the information of R+SMART product in the below link.
 http://support.robotis.com/ko/software/mobile_solution/r+smart/r+smartmain.htm 
 
                 Compatibility
 CM900                  O
 OpenCM9.04             O
 
                 Dynamixel Compatibility
               AX    MX      RX    XL-320    Pro
 CM900          X      X      X        O      X
 OpenCM9.04     X      X      X        O      X
 **** OpenCM 485 EXP board is needed to use 4 pin Dynamixel and Pro Series ****  
 
 created 22 May 2014
 by ROBOTIS CO,.LTD.
 */

#include <OLLO.h>
OLLO myOLLO;

  /* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL1);
Dynamixel Smart(DXL_BUS_SERIAL2);

byte flagState = 0;
word inputTime = 1000;

//위치:2,2, 아이템: 1, 크기: 30, 색상: 흰색
unsigned int opportunityLabel = 18743559;
byte opportunityChance = 3;

//위치:4,2, 아이템: 2, 크기: 30, 색상: 흰색
unsigned int getFlagLabel = 18743817;
byte getFlag = 0;

//목록 설정
byte BlueUP = 3;
byte WhiteUP = 4;
byte BlueDown = 5;
byte WhiteDown = 6;

word getFlagPosition = getFlag * 256;
word opportunityChancePosition = opportunityChance * 256;

//위치:4,3, 아이템: 0, 크기: 50, 색상: 흰색
unsigned int getFlagDisplay = 20054030 + getFlagPosition;
//위치:2,3, 아이템: 0, 크기: 50, 색상: 흰색
unsigned int opportunityChanceDisplay = 20054028 + opportunityChancePosition;

byte intputCheck = 0;

byte BlueSwitch = 0;
byte WhiteSwitch = 0;
byte BeforeBlueSwitch = 0;
byte BeforeWhiteSwitch = 0;
byte randumNum = 0;
int touchTimercount = 0;

void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  Smart.begin(1);

  myOLLO.begin(1,IR_SENSOR);
  myOLLO.begin(4,IR_SENSOR);
   setting();

  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  Dxl.goalPosition(1, 512);
  Dxl.goalPosition(2, 512);
  delay(500);
}

void loop() {
  flagDisplay();
  delay(100);
  sensorCheck();
  faceDisplay();
  scoreDisplay();
}

void setting(){
  //핸드폰 방향 설정
  Smart.writeByte(100, 10010, 2);

  //초기 표시 설정
  Smart.writeDword(100, 10160, opportunityLabel);
  Smart.writeDword(100, 10160, getFlagLabel);

  Smart.writeDword(100, 10170, getFlagDisplay);
  Smart.writeDword(100, 10170, opportunityChanceDisplay);

  //위치:3,3, 아이템: 0, 크기: 0
  Smart.writeWord(100, 10140, 13);
  //위치:3,4, 아이템: 0, 크기: 0
  Smart.writeWord(100, 10140, 18);

  delay(2000);
}
void flagDisplay(){
  flagState = 0;
  intputCheck = 0;
  randumNum = (int)random(5);
  //위치:3,3, 아이템: 3, 크기: 1
  Smart.writeDword(100, 10140, 66317);
  //위치:1,3, 아이템: 0, 크기: 0
  Smart.writeDword(100, 10140, 11);
  //위치:5,3, 아이템: 0, 크기: 0
  Smart.writeDword(100, 10140, 15);

  if(randumNum == 1){
    //위치:1,3, 아이템: 1, 크기: 1
    Smart.writeDword(100, 10140, 65803);
    //TTS 청기내려
    Smart.writeDword(100, 10180, 5);
    while(Smart.readByte(100, 10180)!= 0){
      sensorCheck();
      if(intputCheck == 1){
        break;
      }
    }
  }
  else if(randumNum == 2){
    //위치:1,3, 아이템: 1, 크기: 1
    Smart.writeDword(100, 10140, 65803);
    //TTS 청기올려
    Smart.writeDword(100, 10180, 3);
    while(Smart.readByte(100, 10180)!= 0){
      sensorCheck();
      if(intputCheck == 1){
        break;
      }
    }
  }
  else if(randumNum == 4){
    //위치:5,3, 아이템: 2, 크기: 1
    Smart.writeDword(100, 10140, 66063);
    //TTS 백기내려
    Smart.writeDword(100, 10180, 6);
    while(Smart.readByte(100, 10180)!= 0){
      sensorCheck();
      if(intputCheck == 1){
        break;
      }
    }
  }
  else if(randumNum == 5){
    //위치:1,3, 아이템: 1, 크기: 1
    Smart.writeDword(100, 10140, 66063);
    //TTS 백기올려
    Smart.writeDword(100, 10180, 4);
    while(Smart.readByte(100, 10180)!= 0){
      if(intputCheck == 1){
        break;
      }
    }
  }
}


void intputDetect(){
  if(flagState != 0){

  }
  for(int touchTimercount = 0;touchTimercount< 10000;touchTimercount++){
    sensorCheck();
    if(intputCheck == 1){
      break;
    }
  }
}

void faceDisplay(){
  if(BlueSwitch < 1 && WhiteSwitch < 1)
    if(randumNum == 2 || randumNum == 5)
      flagState = 2;

  if(flagState ==1){
    //위치:3,3, 아이템: 4, 크기: 1
    Smart.writeDword(100, 10140, 66573);
  }
  else if(flagState ==2){
    //위치:3,3, 아이템: 5, 크기: 1
    Smart.writeDword(100, 10140, 66829);
  }
  else{
    //위치:3,3, 아이템: 3, 크기: 1
    Smart.writeDword(100, 10140, 66317);
  }
}
void scoreDisplay(){
  //깃발상태 (0: 대기, 1:캡쳐, 2:미스)
  if(flagState == 1){
    getFlag++;
    inputTime = inputTime - 100;
    if(inputTime <= 0){
      inputTime = 0; 
    }
  }
  else if(flagState == 2){
    opportunityChance--;
  }

  getFlagPosition = getFlag * 256;
  opportunityChancePosition = opportunityChance * 256;

  //위치:4,3, 아이템: 0, 크기: 50, 색상: 흰색
  getFlagDisplay = 20054030 + getFlagPosition;
  //위치:2,3, 아이템: 0, 크기: 50, 색상: 흰색
  opportunityChanceDisplay = 20054028 + opportunityChancePosition;

  Smart.writeDword(100, 10170, getFlagDisplay);
  Smart.writeDword(100, 10170, opportunityChanceDisplay);

  if(opportunityChance == 0)
    gameover();
  if(flagState == 1 || flagState == 2){
    delay(500);
  }
}
void sensorCheck(){
  if(myOLLO.read(1, IR_SENSOR) >= 100)
    BlueSwitch = 1;
  else if(myOLLO.read(4, IR_SENSOR) >= 100)
    WhiteSwitch = 1;
  else{
    BlueSwitch = 0;
    WhiteSwitch = 0;
  }
  if(BlueSwitch <1 && WhiteSwitch < 1){
    flagState = 0;
    BeforeWhiteSwitch = 0;
    BeforeBlueSwitch = 0;
  }
  else{
    if(BeforeWhiteSwitch == 0 && BeforeBlueSwitch == 0){
      if(BlueSwitch > 0){
        if(randumNum == 2){
          flagState = 1; 
        }
        else {
          flagState = 2;
        }
          // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
        Dxl.begin(3);
        Dxl.goalPosition(1, 800);
        delay(400);
        Dxl.goalPosition(1, 512);
        delay(200);
        BeforeBlueSwitch = 1;
        intputCheck = 1;

      }
      else if(WhiteSwitch > 0){
        if( randumNum == 5){
          flagState = 1; 
        }
        else{
          flagState = 2;
        }
          // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
        Dxl.begin(3);
        Dxl.goalPosition(2, 200);
        delay(400);
        Dxl.goalPosition(2, 512); 
        delay(200);
        BeforeWhiteSwitch = 1;
        intputCheck = 1;
      }
    }
  }
}
void gameover(){
  Smart.writeWord(100, 10200, 1);
  while(Smart.readByte(100, 10200) != 0);

  //화면 클리어
  Smart.writeWord(100, 10140, 0);
  Smart.writeWord(100, 10160, 0);
  Smart.writeWord(100, 10170, 0);

  //GamgeOver 표시
  Smart.writeDword(100, 10140, 67085);

  //Retry 표시
  Smart.writeDword(100, 10140, 67346);

  //Retry 깜박임, 클릭시 게임 재시작
  while( Smart.readByte(100, 10310) != 18){
    for(int i = 0;i<=1;i++){
      if(i == 0){
        //위치:3,4, 아이템: 0, 크기: 0
        Smart.writeDword(100, 10140, 18);
      }
      else{
        //위치:3,4, 아이템: 7, 크기: 1
        Smart.writeDword(100, 10140, 67346);
      }
      for(int touchTimercount = 0;touchTimercount< 10000;touchTimercount++){
        if(Smart.readByte(100, 10310) == 18){
          break; 
        }
      }
    }
  }
  setting();
}
