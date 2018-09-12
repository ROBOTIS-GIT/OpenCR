#ifndef TEST_H_
#define TEST_H_

#include "Planar.h"

#define MAX_MOTION_NUM 14

const float tool_position[MAX_MOTION_NUM][3] = {// { x, y, z}
                                                {0.050f,   0.050f, 0.0},  
                                                {-0.050f,  0.050f, 0.0},  
                                                {-0.050f, -0.050f, 0.0},  
                                                {0.050f,  -0.050f, 0.0},  

                                                {0.050f,   0.000f, 0.0},  
                                                {0.000f,   0.050f, 0.0},  
                                                {-0.050f,  0.000f, 0.0},
                                                {0.000f,  -0.050f, 0.0},

                                                {0.025f,  -0.025f, 0.0},
                                                {0.025f,   0.025f, 0.0},
                                                {-0.025f,  0.025f, 0.0},
                                                {-0.025f, -0.025f, 0.0},

                                                {0.025f,  -0.025f, 0.0},
                                                {0.050f,   0.000f, 0.0}  
                                                };

uint8_t motion_cnt = 0;

void test()
{
  if (planar.moving())
  {
    return;
  }
  else
  {
    if (motion_cnt == MAX_MOTION_NUM)
      motion_cnt = 0;

    planar.setMove(TOOL, OM_MATH::makeVector3(
        tool_position[motion_cnt][0] / 2.0, 
        tool_position[motion_cnt][1] / 2.0, 
        tool_position[motion_cnt][2] / 2.0), 
        0.7);
    motion_cnt++;
  }
}

#endif // TEST_H_
