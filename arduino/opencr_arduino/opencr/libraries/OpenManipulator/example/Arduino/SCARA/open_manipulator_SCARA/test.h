#ifndef TEST_H_
#define TEST_H_

#include "SCARA.h"

#define MAX_MOTION_NUM 5

uint8_t motion_erase = 1;
uint8_t motion_page = 11;
uint8_t motion_repeat = 0;

float start_angular_position = 0.0f;
const float move_time = 3.0f;
float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
void *p_init_arg = init_arg;
float radius = 0.020f;

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
        goal_position.push_back(-0.51);
        goal_position.push_back(0.6);
        goal_position.push_back(0.70);

        SCARA.jointMove(goal_position, 3.0f); 
        motion_repeat++;
      }    
      else if (motion_repeat == 2){
        SCARA.toolMove(TOOL, -0.0f);

        std::vector<float> goal_position;
        goal_position.push_back(-0.51);
        goal_position.push_back(0.6);
        goal_position.push_back(0.70);
        SCARA.jointMove(goal_position, 3.0f); 

        motion_repeat++;
      }    
      else if (motion_repeat == 3){
        SCARA.toolMove(TOOL, -0.5f);

        std::vector<float> goal_position;
        goal_position.push_back(-0.51);
        goal_position.push_back(0.6);
        goal_position.push_back(0.70);
        SCARA.jointMove(goal_position, 3.0f); 

        motion_repeat++;
      }    
      else {
        std::vector<float> goal_position;

        if (motion_page == CIRCLE || motion_page == RHOMBUS){
          goal_position.push_back(-1.05);
          goal_position.push_back(0.9);
          goal_position.push_back(0.9);
          SCARA.jointMove(goal_position, 2.0f); 
        }
        else if (motion_page == CIRCLE2) {
          goal_position.push_back(-1.45);
          goal_position.push_back(1.2);
          goal_position.push_back(1.2);
          SCARA.jointMove(goal_position, 2.0f); 
        }

        else {
          goal_position.push_back(-1.2);
          goal_position.push_back(1.0);
          goal_position.push_back(1.0);
          SCARA.jointMove(goal_position, 2.0f); 
        }
        motion_erase = 0;
        motion_repeat = 0;
      }
    }
    else {

      SCARA.toolMove(TOOL, -0.0f);

      if (motion_page == CIRCLE) {
        radius = 0.040f;          
        SCARA.drawInit(CIRCLE, move_time, p_init_arg);
        SCARA.setRadiusForDrawing(CIRCLE, radius);  
        SCARA.setStartPositionForDrawing(CIRCLE, SCARA.getComponentPositionToWorld(TOOL));
        SCARA.setStartAngularPositionForDrawing(CIRCLE, start_angular_position);
        SCARA.draw(CIRCLE);

        motion_erase = 1;
        motion_page++;
        radius = 0.020f;
      }
      
      else if (motion_page == CIRCLE2) { 
        SCARA.drawInit(CIRCLE, move_time, p_init_arg);
        SCARA.setRadiusForDrawing(CIRCLE, radius);  
        SCARA.setStartPositionForDrawing(CIRCLE, SCARA.getComponentPositionToWorld(TOOL));
        SCARA.setStartAngularPositionForDrawing(CIRCLE, start_angular_position);
        SCARA.draw(CIRCLE);

        motion_repeat++;
        start_angular_position = start_angular_position + 2*PI/6;

        if (motion_repeat == 6){
          motion_erase = 1;
          motion_page++;
          motion_repeat = 0;
          start_angular_position = 0;
        }
      } 

      else if (motion_page == RHOMBUS) {
        radius = 0.040f;          
        SCARA.drawInit(RHOMBUS, move_time, p_init_arg);
        SCARA.setRadiusForDrawing(RHOMBUS, radius);  
        SCARA.setStartPositionForDrawing(RHOMBUS, SCARA.getComponentPositionToWorld(TOOL));
        SCARA.setStartAngularPositionForDrawing(RHOMBUS, start_angular_position);
        SCARA.draw(RHOMBUS);

        motion_erase = 1;
        motion_page++;
        radius = 0.020f;
      } 

      else if (motion_page == RHOMBUS2) {
        SCARA.drawInit(RHOMBUS, move_time, p_init_arg);
        SCARA.setRadiusForDrawing(RHOMBUS, radius);  
        SCARA.setStartPositionForDrawing(RHOMBUS, SCARA.getComponentPositionToWorld(TOOL));
        SCARA.setStartAngularPositionForDrawing(RHOMBUS, start_angular_position);
        SCARA.draw(RHOMBUS);

        radius += 0.007f;
        motion_repeat++;

        if (motion_repeat == 3){
          motion_erase = 1;
          motion_page++;
          motion_repeat = 0;
          radius = 0.020f;
        }
      } 

      else if (motion_page == HEART) { 
        radius = 0.050f;
        SCARA.drawInit(HEART, move_time, p_init_arg);
        SCARA.setRadiusForDrawing(HEART, radius);  
        SCARA.setStartPositionForDrawing(HEART, SCARA.getComponentPositionToWorld(TOOL));
        SCARA.setStartAngularPositionForDrawing(HEART, start_angular_position);
        SCARA.draw(HEART);

        motion_erase = 1;
        motion_page++;
        radius = 0.020f;
      } 

      // else if (motion_page == RHOMBUS2) { 
      //   SCARA.drawInit(RHOMBUS, move_time, p_init_arg);
      //   SCARA.setRadiusForDrawing(RHOMBUS, radius);  
      //   SCARA.setStartPositionForDrawing(RHOMBUS, SCARA.getComponentPositionToWorld(TOOL));
      //   SCARA.setStartAngularPositionForDrawing(RHOMBUS, start_angular_position);
      //   SCARA.draw(RHOMBUS);

      //   motion_repeat++;
      //   start_angular_position = start_angular_position + PI/4;

      //   if (motion_repeat == 8){
      //     motion_erase = 1;
      //     motion_page++;
      //     motion_repeat = 0;
      //     start_angular_position = 0;
      //   }
      // } 

      // else if (motion_page == HEART2) { 
      //   SCARA.drawInit(HEART, move_time, p_init_arg);
      //   SCARA.setRadiusForDrawing(HEART, radius);  
      //   SCARA.setStartPositionForDrawing(HEART, SCARA.getComponentPositionToWorld(TOOL));
      //   SCARA.setStartAngularPositionForDrawing(HEART, start_angular_position);
      //   SCARA.draw(HEART);

      //   motion_repeat++;
      //   start_angular_position = start_angular_position + PI/4;

      //   if (motion_repeat == 8){
      //     motion_erase = 1;
      //     motion_page++;
      //     motion_repeat = 0;
      //     start_angular_position = 0;
      //   }
      // } 

      else
        motion_page = 11;
    }
  }
}

#endif // TEST_H_
