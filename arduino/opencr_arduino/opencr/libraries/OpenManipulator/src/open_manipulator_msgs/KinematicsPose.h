#ifndef _ROS_open_manipulator_msgs_KinematicsPose_h
#define _ROS_open_manipulator_msgs_KinematicsPose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace open_manipulator_msgs
{

  class KinematicsPose : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef float _max_accelerations_scaling_factor_type;
      _max_accelerations_scaling_factor_type max_accelerations_scaling_factor;
      typedef float _max_velocity_scaling_factor_type;
      _max_velocity_scaling_factor_type max_velocity_scaling_factor;
      typedef float _tolerance_type;
      _tolerance_type tolerance;

    KinematicsPose():
      pose(),
      max_accelerations_scaling_factor(0),
      max_velocity_scaling_factor(0),
      tolerance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_accelerations_scaling_factor);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_velocity_scaling_factor);
      offset += serializeAvrFloat64(outbuffer + offset, this->tolerance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_accelerations_scaling_factor));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_velocity_scaling_factor));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tolerance));
     return offset;
    }

    const char * getType(){ return "open_manipulator_msgs/KinematicsPose"; };
    const char * getMD5(){ return "bad8d5def2efabb0336490f8e9f6f2e2"; };

  };

}
#endif