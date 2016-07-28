#include <OLLO.h>

#define INTERVAL 100000 //micros

OLLO myOLLO;
HardwareTimer Timer(1);
long before_start_time = 0; //前回の開始時間

void setup() {
  
  myOLLO.begin(1);
  SerialUSB.begin();

  Timer.pause();
  Timer.setPeriod(INTERVAL); //タイマ割り込み時間間隔を設定
  Timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  Timer.setCompare(TIMER_CH1, 1);
  Timer.attachInterrupt(TIMER_CH1, getDMSValue); //割り込む関数を指定
  Timer.refresh();
  Timer.resume();
}

void loop() {
  // Nothing! It's all in the handler_led() interrupt:
}

void getDMSValue(void) {
  long start_time = 0; //処理開始時間
  long interval_time = 0; //前回の割り込みからの経過時間
  int DMS_value = 0;
  int tmp =0;
    
  start_time = micros(); //開始時間取得
  interval_time = start_time - before_start_time;
  before_start_time = start_time; //更新
  
  DMS_value = myOLLO.read(1); //距離センサ値取得
  
  //距離が近づいてきたら
  if(DMS_value >= 1000){
   for(int i=0; i< 100000; i++) {
      tmp = tmp + i - tmp * i / tmp;
   }
  }
  
  //出力
  SerialUSB.print("[INTERVAL=");
  SerialUSB.print(INTERVAL); 
  SerialUSB.print("] "); 
  SerialUSB.print("DMS=");
  SerialUSB.print(DMS_value); 
  SerialUSB.print(", start_time=");
  SerialUSB.print(start_time);
  SerialUSB.print(", interval_time=");
  SerialUSB.print(interval_time);
  SerialUSB.print(", exec_time=");
  SerialUSB.println(micros()- start_time); //処理時間計算
 
}


