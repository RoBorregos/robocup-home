#ifndef _ROS_nav_2d_msgs_Twist2DStamped_h
#define _ROS_nav_2d_msgs_Twist2DStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "nav_2d_msgs/Twist2D.h"

namespace nav_2d_msgs
{

  class Twist2DStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef nav_2d_msgs::Twist2D _velocity_type;
      _velocity_type velocity;

    Twist2DStamped():
      header(),
      velocity()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "nav_2d_msgs/Twist2DStamped"; };
    virtual const char * getMD5() override { return "dd276ca6100434e23de2f104f6c317c2"; };

  };

}
#endif
