#ifndef TEST_H_
#define TEST_H_

#include "SCARA.h"

#define MAX_MOTION_NUM 5

const float tool_position[MAX_MOTION_NUM][3] = {// { x, y, z}
                                                {0.050f,   0.050f, 0.0},  
                                                {-0.050f,  0.050f, 0.0},  
                                                {-0.050f, -0.050f, 0.0},  
                                                {0.050f,  -0.050f, 0.0},  
                                                {0.050f,  -0.050f, 0.0}};  
uint8_t motion_cnt = 0;


void test()
{
  if (SCARA.drawing())
  {
    Serial.println("now drawing");
    return;
  }
  else
  {      
    Serial.println("next circle");
    // if (motion_cnt == MAX_MOTION_NUM){
    //     std::vector<float> goal_position;
    //     motion_cnt = 0;   
    //     goal_position.push_back(-1.3);
    //     goal_position.push_back(1.0);
    //     goal_position.push_back(1.5);
    //     SCARA.jointMove(goal_position, 1.0f);
    // }

    const float move_time = 5.0f;
    float init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
    void *p_init_arg = init_arg;
    static float radius = 0.015f;

    SCARA.drawInit(CIRCLE, move_time, p_init_arg);
    SCARA.setRadiusForDrawing(CIRCLE, radius);  
    SCARA.setStartPositionForDrawing(CIRCLE, SCARA.getComponentPositionToWorld(TOOL));
    SCARA.draw(CIRCLE);

    //   SCARA.drawInit(RHOMBUS, move_time, p_init_arg);
    //   SCARA.setRadiusForDrawing(RHOMBUS, radius);  
    //   SCARA.setStartPositionForDrawing(RHOMBUS, SCARA.getComponentPositionToWorld(TOOL));
    //   SCARA.draw(RHOMBUS);

    //   SCARA.drawInit(HEART, move_time, p_init_arg);
    //   SCARA.setRadiusForDrawing(HEART, radius);  
    //   SCARA.setStartPositionForDrawing(HEART, SCARA.getComponentPositionToWorld(TOOL));
    //   SCARA.draw(HEART);
    motion_cnt = 1;    
  }
  // motion_cnt++;    
}

#endif // TEST_H_
