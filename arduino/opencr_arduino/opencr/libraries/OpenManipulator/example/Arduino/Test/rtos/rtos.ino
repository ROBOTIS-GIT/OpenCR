#include <RTOS.h>
#include <OpenManipulator.h>

osThreadId thread_id_loop;
osThreadId thread_id_led;

OMDynamixel<1,1000000> omDynamixel;

static void Thread_Loop(void const *argument)
{
  (void) argument;

  for(;;)
  {
    loop();
  }
}

void setup() 
{
  Serial.begin(115200);

  omDynamixel.init();
  omDynamixel.setPositionControlMode(1);
  omDynamixel.setEnable(1);

  // define thread
  osThreadDef(THREAD_NAME_LOOP, Thread_Loop, osPriorityNormal, 0, 1024);
  osThreadDef(THREAD_NAME_LED,  Thread_Led,  osPriorityNormal, 0, 1024);

  // create thread
  thread_id_loop = osThreadCreate(osThread(THREAD_NAME_LOOP), NULL);
  thread_id_led  = osThreadCreate(osThread(THREAD_NAME_LED), NULL);

  // start kernel
  osKernelStart();
}

void loop() 
{
  float* angle = omDynamixel.getAngle();
  Serial.print("angle : "); Serial.println(angle[0]);

  omDynamixel.setAngle(1, 1000);
  // static uint32_t cnt = 0;
  
  // Serial.print("RTOS Cnt : ");
  // Serial.println(cnt++);
  osDelay(100);    
}

static void Thread_Led(void const *argument)
{
  (void) argument;

  // pinMode(13, OUTPUT);

  for(;;)
  {
    // digitalWrite(13, !digitalRead(13));
    // osDelay(300);

    // pose  = getPose()
    // state = getState()
  }
}
