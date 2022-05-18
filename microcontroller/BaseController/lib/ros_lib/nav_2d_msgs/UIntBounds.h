#ifndef _ROS_nav_2d_msgs_UIntBounds_h
#define _ROS_nav_2d_msgs_UIntBounds_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nav_2d_msgs
{

  class UIntBounds : public ros::Msg
  {
    public:
      typedef uint32_t _min_x_type;
      _min_x_type min_x;
      typedef uint32_t _min_y_type;
      _min_y_type min_y;
      typedef uint32_t _max_x_type;
      _max_x_type max_x;
      typedef uint32_t _max_y_type;
      _max_y_type max_y;

    UIntBounds():
      min_x(0),
      min_y(0),
      max_x(0),
      max_y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->min_x >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->min_x >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->min_x >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->min_x >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_x);
      *(outbuffer + offset + 0) = (this->min_y >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->min_y >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->min_y >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->min_y >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_y);
      *(outbuffer + offset + 0) = (this->max_x >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_x >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->max_x >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->max_x >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_x);
      *(outbuffer + offset + 0) = (this->max_y >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_y >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->max_y >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->max_y >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->min_x =  ((uint32_t) (*(inbuffer + offset)));
      this->min_x |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->min_x |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->min_x |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->min_x);
      this->min_y =  ((uint32_t) (*(inbuffer + offset)));
      this->min_y |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->min_y |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->min_y |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->min_y);
      this->max_x =  ((uint32_t) (*(inbuffer + offset)));
      this->max_x |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->max_x |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->max_x |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->max_x);
      this->max_y =  ((uint32_t) (*(inbuffer + offset)));
      this->max_y |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->max_y |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->max_y |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->max_y);
     return offset;
    }

    virtual const char * getType() override { return "nav_2d_msgs/UIntBounds"; };
    virtual const char * getMD5() override { return "32cc77a889ecfebef439f3192db73e63"; };

  };

}
#endif
