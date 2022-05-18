#ifndef _ROS_nav_2d_msgs_Point2D_h
#define _ROS_nav_2d_msgs_Point2D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nav_2d_msgs
{

  class Point2D : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;

    Point2D():
      x(0),
      y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
     return offset;
    }

    virtual const char * getType() override { return "nav_2d_msgs/Point2D"; };
    virtual const char * getMD5() override { return "209f516d3eb691f0663e25cb750d67c1"; };

  };

}
#endif
