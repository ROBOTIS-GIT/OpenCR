#ifndef _ROS_turtlebot3_msgs_SensorState_h
#define _ROS_turtlebot3_msgs_SensorState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace turtlebot3_msgs
{

  class SensorState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _bumper_type;
      _bumper_type bumper;
      typedef uint8_t _cliff_type;
      _cliff_type cliff;
      typedef uint8_t _button_type;
      _button_type button;
      typedef bool _torque_type;
      _torque_type torque;
      typedef int32_t _left_encoder_type;
      _left_encoder_type left_encoder;
      typedef int32_t _right_encoder_type;
      _right_encoder_type right_encoder;
      typedef float _battery_type;
      _battery_type battery;
      enum { BUMPER_RIGHT =  1 };
      enum { BUMPER_CENTER =  2 };
      enum { BUMPER_LEFT =  4 };
      enum { CLIFF_RIGHT =  1 };
      enum { CLIFF_CENTER =  2 };
      enum { CLIFF_LEFT =  4 };
      enum { BUTTON0 =  1 };
      enum { BUTTON1 =  2 };
      enum { BUTTON2 =  4 };
      enum { ERROR_LEFT_MOTOR =  1 };
      enum { ERROR_RIGHT_MOTOR =  2 };
      enum { TORQUE_ON =  1 };
      enum { TORQUE_OFF =  2 };

    SensorState():
      header(),
      bumper(0),
      cliff(0),
      button(0),
      torque(0),
      left_encoder(0),
      right_encoder(0),
      battery(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->bumper >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bumper);
      *(outbuffer + offset + 0) = (this->cliff >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cliff);
      *(outbuffer + offset + 0) = (this->button >> (8 * 0)) & 0xFF;
      offset += sizeof(this->button);
      union {
        bool real;
        uint8_t base;
      } u_torque;
      u_torque.real = this->torque;
      *(outbuffer + offset + 0) = (u_torque.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->torque);
      union {
        int32_t real;
        uint32_t base;
      } u_left_encoder;
      u_left_encoder.real = this->left_encoder;
      *(outbuffer + offset + 0) = (u_left_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_encoder.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_encoder);
      union {
        int32_t real;
        uint32_t base;
      } u_right_encoder;
      u_right_encoder.real = this->right_encoder;
      *(outbuffer + offset + 0) = (u_right_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_encoder.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_encoder);
      union {
        float real;
        uint32_t base;
      } u_battery;
      u_battery.real = this->battery;
      *(outbuffer + offset + 0) = (u_battery.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->bumper =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->bumper);
      this->cliff =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cliff);
      this->button =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->button);
      union {
        bool real;
        uint8_t base;
      } u_torque;
      u_torque.base = 0;
      u_torque.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->torque = u_torque.real;
      offset += sizeof(this->torque);
      union {
        int32_t real;
        uint32_t base;
      } u_left_encoder;
      u_left_encoder.base = 0;
      u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_encoder = u_left_encoder.real;
      offset += sizeof(this->left_encoder);
      union {
        int32_t real;
        uint32_t base;
      } u_right_encoder;
      u_right_encoder.base = 0;
      u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_encoder = u_right_encoder.real;
      offset += sizeof(this->right_encoder);
      union {
        float real;
        uint32_t base;
      } u_battery;
      u_battery.base = 0;
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery = u_battery.real;
      offset += sizeof(this->battery);
     return offset;
    }

    const char * getType(){ return "turtlebot3_msgs/SensorState"; };
    const char * getMD5(){ return "d537ed7b8d95065b6c83830430b93911"; };

  };

}
#endif