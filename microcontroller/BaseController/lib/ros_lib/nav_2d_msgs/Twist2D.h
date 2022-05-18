#ifndef _ROS_nav_2d_msgs_Twist2D_h
#define _ROS_nav_2d_msgs_Twist2D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nav_2d_msgs
{

  class Twist2D : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _theta_type;
      _theta_type theta;

    Twist2D():
      x(0),
      y(0),
      theta(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->theta);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->theta));
     return offset;
    }

    virtual const char * getType() override { return "nav_2d_msgs/Twist2D"; };
    virtual const char * getMD5() override { return "938fa65709584ad8e77d238529be13b8"; };

  };

}
#endif
