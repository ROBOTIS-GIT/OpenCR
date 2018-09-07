#ifndef TEST_H_
#define TEST_H_

#include "SCARA.h"
#include <OMDebug.h>

#define MAX_MOTION_NUM 5

uint8_t motion_erase = 1;
uint8_t motion_page = 11;
uint8_t motion_repeat = 0;

float start_angular_position = 0.0f;
const float move_time = 5.0f;
float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
void *p_init_arg = init_arg;
float radius = 0.010f;

void test()
{
  if (SCARA.moving() || SCARA.drawing()) {
    return;
  }
  else {
    if (motion_erase == 1){
      if (motion_repeat == 0){
        SCARA.toolMove(TOOL, -0.5f);
        motion_repeat++;
      }    
      else if (motion_repeat == 1){
        std::vector<float> goal_position;
        goal_position.push_back(-0.521);
        goal_position.push_back(0.6);
        goal_position.push_back(0.70);

        SCARA.jointMove(goal_position, 3.0f); 
        motion_repeat++;
      }    
      else if (motion_repeat == 2){
        SCARA.toolMove(TOOL, -0.05f);

        std::vector<float> goal_position;
        goal_position.push_back(-0.521);
        goal_position.push_back(0.6);
        goal_position.push_back(0.70);
        SCARA.jointMove(goal_position, 3.0f); 

        motion_repeat++;
      }    
      else if (motion_repeat == 3){
        SCARA.toolMove(TOOL, -0.5f);

        std::vector<float> goal_position;
        goal_position.push_back(-0.521);
        goal_position.push_back(0.6);
        goal_position.push_back(0.70);
        SCARA.jointMove(goal_position, 3.0f); 

        motion_repeat++;
      }    
      else {
        std::vector<float> goal_position;
        goal_position.push_back(-0.9);
        goal_position.push_back(0.5);
        goal_position.push_back(1.3);
        SCARA.jointMove(goal_position, 2.0f); 

        motion_erase = 0;
        motion_repeat = 0;
      }
    }
    else {

        SCARA.toolMove(TOOL, -0.05f);

        if (motion_page == CIRCLE) {
          
          SCARA.drawInit(CIRCLE, move_time, p_init_arg);
          SCARA.setRadiusForDrawing(CIRCLE, radius);  

          // PRINT::VECTOR(SCARA.getComponentPositionToWorld(TOOL)); 
          // Serial.println(SCARA.getComponentPositionToWorld(TOOL)[0]);
          // Serial.println(SCARA.getComponentPositionToWorld(TOOL)[1]);
          // Serial.println(SCARA.getComponentPositionToWorld(TOOL)[2]);

          SCARA.setStartPositionForDrawing(CIRCLE, SCARA.getComponentPositionToWorld(TOOL));
          SCARA.setStartAngularPositionForDrawing(CIRCLE, start_angular_position);
          SCARA.draw(CIRCLE);

          radius += 0.002f;
          motion_repeat++;
          
          if (motion_repeat == 4){
            motion_erase = 1;
            motion_page++;
            motion_repeat = 0;
            radius = 0.015f;
        }
      } 
      else if (motion_page == RHOMBUS) {
        SCARA.drawInit(RHOMBUS, move_time, p_init_arg);
        SCARA.setRadiusForDrawing(RHOMBUS, radius);  
        SCARA.setStartPositionForDrawing(RHOMBUS, SCARA.getComponentPositionToWorld(TOOL));
        SCARA.setStartAngularPositionForDrawing(RHOMBUS, start_angular_position);
        SCARA.draw(RHOMBUS);

        radius += 0.002f;
        motion_repeat++;

        if (motion_repeat == 4){
          motion_erase = 1;
          motion_page++;
          motion_repeat = 0;
          radius = 0.015f;
        }
      } 
      else if (motion_page == HEART) { 
        // radius = 0.030f;
        SCARA.drawInit(HEART, move_time, p_init_arg);
        SCARA.setRadiusForDrawing(HEART, radius);  
        SCARA.setStartPositionForDrawing(HEART, SCARA.getComponentPositionToWorld(TOOL));
        SCARA.setStartAngularPositionForDrawing(HEART, start_angular_position);
        SCARA.draw(HEART);

        radius += 0.002f;
        motion_repeat++;

        if (motion_repeat == 4){
          motion_erase = 1;
          motion_page++;
          motion_repeat = 0;
          radius = 0.015f;
        }
      } 

      // else if (motion_page == CIRCLE2) { 
      //   SCARA.drawInit(CIRCLE, move_time, p_init_arg);
      //   SCARA.setRadiusForDrawing(CIRCLE, radius);  
      //   SCARA.setStartPositionForDrawing(CIRCLE, SCARA.getComponentPositionToWorld(TOOL));
      //   SCARA.setStartAngularPositionForDrawing(CIRCLE, start_angular_position);
      //   SCARA.draw(CIRCLE);

      //   motion_repeat++;
      //   start_angular_position = start_angular_position + PI/4;

      //   if (motion_repeat == 6){
      //     motion_erase = 1;
      //     motion_page++;
      //     motion_repeat = 0;
      //     start_angular_position = 0;
      //   }
      // } 

      else
        motion_page = 12;

    }
  }
}

#endif // TEST_H_
