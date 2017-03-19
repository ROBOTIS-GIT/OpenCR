#include <RTOS.h>


osThreadId thread_id_loop;
osThreadId thread_id_led;



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
  static uint32_t cnt = 0;
  
  Serial.print("RTOS Cnt : ");
  Serial.println(cnt++);
  osDelay(100);  
}

static void Thread_Led(void const *argument)
{
  (void) argument;


  pinMode(13, OUTPUT);

  for(;;)
  {
    digitalWrite(13, !digitalRead(13));
    osDelay(300);
  }
}

