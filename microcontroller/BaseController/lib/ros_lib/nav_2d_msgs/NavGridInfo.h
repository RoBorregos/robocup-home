#ifndef _ROS_nav_2d_msgs_NavGridInfo_h
#define _ROS_nav_2d_msgs_NavGridInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nav_2d_msgs
{

  class NavGridInfo : public ros::Msg
  {
    public:
      typedef uint32_t _width_type;
      _width_type width;
      typedef uint32_t _height_type;
      _height_type height;
      typedef float _resolution_type;
      _resolution_type resolution;
      typedef const char* _frame_id_type;
      _frame_id_type frame_id;
      typedef float _origin_x_type;
      _origin_x_type origin_x;
      typedef float _origin_y_type;
      _origin_y_type origin_y;

    NavGridInfo():
      width(0),
      height(0),
      resolution(0),
      frame_id(""),
      origin_x(0),
      origin_y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->width >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->width >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->width >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->width >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      *(outbuffer + offset + 0) = (this->height >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->height >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->height >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->height >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      offset += serializeAvrFloat64(outbuffer + offset, this->resolution);
      uint32_t length_frame_id = strlen(this->frame_id);
      varToArr(outbuffer + offset, length_frame_id);
      offset += 4;
      memcpy(outbuffer + offset, this->frame_id, length_frame_id);
      offset += length_frame_id;
      offset += serializeAvrFloat64(outbuffer + offset, this->origin_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->origin_y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->width =  ((uint32_t) (*(inbuffer + offset)));
      this->width |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->width |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->width |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->width);
      this->height =  ((uint32_t) (*(inbuffer + offset)));
      this->height |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->height |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->height |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->height);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->resolution));
      uint32_t length_frame_id;
      arrToVar(length_frame_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_frame_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_frame_id-1]=0;
      this->frame_id = (char *)(inbuffer + offset-1);
      offset += length_frame_id;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->origin_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->origin_y));
     return offset;
    }

    virtual const char * getType() override { return "nav_2d_msgs/NavGridInfo"; };
    virtual const char * getMD5() override { return "061e7a10093a3d95bf6b212dff9d9715"; };

  };

}
#endif
