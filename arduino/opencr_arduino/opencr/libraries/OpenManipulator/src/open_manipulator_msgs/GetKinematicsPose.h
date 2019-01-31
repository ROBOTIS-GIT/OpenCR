#ifndef _ROS_SERVICE_GetKinematicsPose_h
#define _ROS_SERVICE_GetKinematicsPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "open_manipulator_msgs/KinematicsPose.h"
#include "std_msgs/Header.h"

namespace open_manipulator_msgs
{

static const char GETKINEMATICSPOSE[] = "open_manipulator_msgs/GetKinematicsPose";

  class GetKinematicsPoseRequest : public ros::Msg
  {
    public:
      typedef const char* _planning_group_type;
      _planning_group_type planning_group;
      typedef const char* _end_effector_name_type;
      _end_effector_name_type end_effector_name;

    GetKinematicsPoseRequest():
      planning_group(""),
      end_effector_name("")
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
      uint32_t length_end_effector_name = strlen(this->end_effector_name);
      varToArr(outbuffer + offset, length_end_effector_name);
      offset += 4;
      memcpy(outbuffer + offset, this->end_effector_name, length_end_effector_name);
      offset += length_end_effector_name;
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
      uint32_t length_end_effector_name;
      arrToVar(length_end_effector_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_end_effector_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_end_effector_name-1]=0;
      this->end_effector_name = (char *)(inbuffer + offset-1);
      offset += length_end_effector_name;
     return offset;
    }

    const char * getType(){ return GETKINEMATICSPOSE; };
    const char * getMD5(){ return "14dd5674451c0fe6eacac0ded7197f30"; };

  };

  class GetKinematicsPoseResponse : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef open_manipulator_msgs::KinematicsPose _kinematics_pose_type;
      _kinematics_pose_type kinematics_pose;

    GetKinematicsPoseResponse():
      header(),
      kinematics_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->kinematics_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->kinematics_pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETKINEMATICSPOSE; };
    const char * getMD5(){ return "3b64b73433e2775c9c4b7e1a00dd6995"; };

  };

  class GetKinematicsPose {
    public:
    typedef GetKinematicsPoseRequest Request;
    typedef GetKinematicsPoseResponse Response;
  };

}
#endif
