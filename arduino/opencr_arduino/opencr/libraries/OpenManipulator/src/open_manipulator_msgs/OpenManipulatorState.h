#ifndef _ROS_open_manipulator_msgs_OpenManipulatorState_h
#define _ROS_open_manipulator_msgs_OpenManipulatorState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace open_manipulator_msgs
{

  class OpenManipulatorState : public ros::Msg
  {
    public:
      typedef const char* _open_manipulator_moving_state_type;
      _open_manipulator_moving_state_type open_manipulator_moving_state;
      typedef const char* _open_manipulator_actuator_state_type;
      _open_manipulator_actuator_state_type open_manipulator_actuator_state;
      const char* IS_MOVING =  "IS_MOVING";
      const char* STOPPED =  "STOPPED";
      const char* ACTUATOR_ENABLED =  "ACTUATOR_ENABLED";
      const char* ACTUATOR_DISABLED =  "ACTUATOR_DISABLED";

    OpenManipulatorState():
      open_manipulator_moving_state(""),
      open_manipulator_actuator_state("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_open_manipulator_moving_state = strlen(this->open_manipulator_moving_state);
      varToArr(outbuffer + offset, length_open_manipulator_moving_state);
      offset += 4;
      memcpy(outbuffer + offset, this->open_manipulator_moving_state, length_open_manipulator_moving_state);
      offset += length_open_manipulator_moving_state;
      uint32_t length_open_manipulator_actuator_state = strlen(this->open_manipulator_actuator_state);
      varToArr(outbuffer + offset, length_open_manipulator_actuator_state);
      offset += 4;
      memcpy(outbuffer + offset, this->open_manipulator_actuator_state, length_open_manipulator_actuator_state);
      offset += length_open_manipulator_actuator_state;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_open_manipulator_moving_state;
      arrToVar(length_open_manipulator_moving_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_open_manipulator_moving_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_open_manipulator_moving_state-1]=0;
      this->open_manipulator_moving_state = (char *)(inbuffer + offset-1);
      offset += length_open_manipulator_moving_state;
      uint32_t length_open_manipulator_actuator_state;
      arrToVar(length_open_manipulator_actuator_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_open_manipulator_actuator_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_open_manipulator_actuator_state-1]=0;
      this->open_manipulator_actuator_state = (char *)(inbuffer + offset-1);
      offset += length_open_manipulator_actuator_state;
     return offset;
    }

    const char * getType(){ return "open_manipulator_msgs/OpenManipulatorState"; };
    const char * getMD5(){ return "35c95327a0dcb52791892bac52df33e8"; };

  };

}
#endif