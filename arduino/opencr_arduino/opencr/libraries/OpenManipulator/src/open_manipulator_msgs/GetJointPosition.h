#ifndef _ROS_SERVICE_GetJointPosition_h
#define _ROS_SERVICE_GetJointPosition_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "open_manipulator_msgs/JointPosition.h"

namespace open_manipulator_msgs
{

static const char GETJOINTPOSITION[] = "open_manipulator_msgs/GetJointPosition";

  class GetJointPositionRequest : public ros::Msg
  {
    public:
      typedef const char* _planning_group_type;
      _planning_group_type planning_group;

    GetJointPositionRequest():
      planning_group("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_planning_group = strlen(this->planning_group);
      varToArr(outbuffer + offset, length_planning_group);
      offset += 4;
      memcpy(outbuffer + offset, this->planning_group, length_planning_group);
      offset += length_planning_group;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_planning_group;
      arrToVar(length_planning_group, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_planning_group; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_planning_group-1]=0;
      this->planning_group = (char *)(inbuffer + offset-1);
      offset += length_planning_group;
     return offset;
    }

    const char * getType(){ return GETJOINTPOSITION; };
    const char * getMD5(){ return "6b02e06b167eb20b51185dc7d0b638aa"; };

  };

  class GetJointPositionResponse : public ros::Msg
  {
    public:
      typedef open_manipulator_msgs::JointPosition _joint_position_type;
      _joint_position_type joint_position;

    GetJointPositionResponse():
      joint_position()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->joint_position.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->joint_position.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETJOINTPOSITION; };
    const char * getMD5(){ return "e1f1ee99b5e77308297dc4eeedd305d4"; };

  };

  class GetJointPosition {
    public:
    typedef GetJointPositionRequest Request;
    typedef GetJointPositionResponse Response;
  };

}
#endif
