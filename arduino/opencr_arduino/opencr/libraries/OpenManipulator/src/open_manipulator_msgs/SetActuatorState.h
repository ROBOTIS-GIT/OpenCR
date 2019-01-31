#ifndef _ROS_SERVICE_SetActuatorState_h
#define _ROS_SERVICE_SetActuatorState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace open_manipulator_msgs
{

static const char SETACTUATORSTATE[] = "open_manipulator_msgs/SetActuatorState";

  class SetActuatorStateRequest : public ros::Msg
  {
    public:
      typedef bool _set_actuator_state_type;
      _set_actuator_state_type set_actuator_state;

    SetActuatorStateRequest():
      set_actuator_state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_set_actuator_state;
      u_set_actuator_state.real = this->set_actuator_state;
      *(outbuffer + offset + 0) = (u_set_actuator_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->set_actuator_state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_set_actuator_state;
      u_set_actuator_state.base = 0;
      u_set_actuator_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->set_actuator_state = u_set_actuator_state.real;
      offset += sizeof(this->set_actuator_state);
     return offset;
    }

    const char * getType(){ return SETACTUATORSTATE; };
    const char * getMD5(){ return "0a8a1fa84fab126c7567fbb489a23ea5"; };

  };

  class SetActuatorStateResponse : public ros::Msg
  {
    public:
      typedef bool _is_planned_type;
      _is_planned_type is_planned;

    SetActuatorStateResponse():
      is_planned(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_planned;
      u_is_planned.real = this->is_planned;
      *(outbuffer + offset + 0) = (u_is_planned.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_planned);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_planned;
      u_is_planned.base = 0;
      u_is_planned.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_planned = u_is_planned.real;
      offset += sizeof(this->is_planned);
     return offset;
    }

    const char * getType(){ return SETACTUATORSTATE; };
    const char * getMD5(){ return "2638cc2443b1469b0e9e152083d7128d"; };

  };

  class SetActuatorState {
    public:
    typedef SetActuatorStateRequest Request;
    typedef SetActuatorStateResponse Response;
  };

}
#endif
