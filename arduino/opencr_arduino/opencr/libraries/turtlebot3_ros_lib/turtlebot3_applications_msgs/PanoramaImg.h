#ifndef _ROS_turtlebot3_applications_msgs_PanoramaImg_h
#define _ROS_turtlebot3_applications_msgs_PanoramaImg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"

namespace turtlebot3_applications_msgs
{

  class PanoramaImg : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _pano_id_type;
      _pano_id_type pano_id;
      typedef float _latitude_type;
      _latitude_type latitude;
      typedef float _longitude_type;
      _longitude_type longitude;
      typedef float _heading_type;
      _heading_type heading;
      typedef const char* _geo_tag_type;
      _geo_tag_type geo_tag;
      typedef sensor_msgs::Image _image_type;
      _image_type image;

    PanoramaImg():
      header(),
      pano_id(""),
      latitude(0),
      longitude(0),
      heading(0),
      geo_tag(""),
      image()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_pano_id = strlen(this->pano_id);
      varToArr(outbuffer + offset, length_pano_id);
      offset += 4;
      memcpy(outbuffer + offset, this->pano_id, length_pano_id);
      offset += length_pano_id;
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->heading);
      uint32_t length_geo_tag = strlen(this->geo_tag);
      varToArr(outbuffer + offset, length_geo_tag);
      offset += 4;
      memcpy(outbuffer + offset, this->geo_tag, length_geo_tag);
      offset += length_geo_tag;
      offset += this->image.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_pano_id;
      arrToVar(length_pano_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_pano_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_pano_id-1]=0;
      this->pano_id = (char *)(inbuffer + offset-1);
      offset += length_pano_id;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->heading));
      uint32_t length_geo_tag;
      arrToVar(length_geo_tag, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_geo_tag; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_geo_tag-1]=0;
      this->geo_tag = (char *)(inbuffer + offset-1);
      offset += length_geo_tag;
      offset += this->image.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "turtlebot3_applications_msgs/PanoramaImg"; };
    const char * getMD5(){ return "aedf66295b374a7249a786af27aecc87"; };

  };

}
#endif