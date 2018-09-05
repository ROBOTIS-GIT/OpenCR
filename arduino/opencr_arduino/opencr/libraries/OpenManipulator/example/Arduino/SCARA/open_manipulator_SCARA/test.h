#ifndef TEST_H_
#define TEST_H_

#include "SCARA.h"

#define MAX_MOTION_NUM 5

uint8_t motion_page = 11;
uint8_t motion_repeat = 0;
uint8_t motion_erase = 0;


const float move_time = 5.0f;
float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
void *p_init_arg = init_arg;
float radius = 0.015f;

void test()
{
  if (SCARA.drawing())
  {
    return;
  }
  else
  {      
    if (motion_erase == 1){

    }
    else if (motion_page == CIRCLE) {
      SCARA.drawInit(CIRCLE, move_time, p_init_arg);
      SCARA.setRadiusForDrawing(CIRCLE, radius);  
      SCARA.setStartPositionForDrawing(CIRCLE, SCARA.getComponentPositionToWorld(TOOL));
      SCARA.draw(CIRCLE);

      radius += 0.002f;
      motion_repeat++;
      
      if (motion_repeat == 4){
        motion_page++;
        motion_repeat = 0;
        radius = 0.015f;
      }
    } 
    else if (motion_page == RHOMBUS) {
      SCARA.drawInit(RHOMBUS, move_time, p_init_arg);
      SCARA.setRadiusForDrawing(RHOMBUS, radius);  
      SCARA.setStartPositionForDrawing(RHOMBUS, SCARA.getComponentPositionToWorld(TOOL));
      SCARA.draw(RHOMBUS);

      radius += 0.002f;
      motion_repeat++;

      if (motion_repeat == 4){
        motion_page++;
        motion_repeat = 0;
        radius = 0.015f;
      }
    } 
    else if (motion_page == HEART) { 
      SCARA.drawInit(HEART, move_time, p_init_arg);
      SCARA.setRadiusForDrawing(HEART, radius);  
      SCARA.setStartPositionForDrawing(HEART, SCARA.getComponentPositionToWorld(TOOL));
      SCARA.draw(HEART);

      radius += 0.002f;
      motion_repeat++;

      if (motion_repeat == 4){
        motion_page++;
        motion_repeat = 0;
        radius = 0.015f;
      }
    } 
    else
      motion_page = 11;
  }
}

#endif // TEST_H_
