#ifndef _ROS_nav_2d_msgs_Polygon2DStamped_h
#define _ROS_nav_2d_msgs_Polygon2DStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "nav_2d_msgs/Polygon2D.h"

namespace nav_2d_msgs
{

  class Polygon2DStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef nav_2d_msgs::Polygon2D _polygon_type;
      _polygon_type polygon;

    Polygon2DStamped():
      header(),
      polygon()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->polygon.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->polygon.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "nav_2d_msgs/Polygon2DStamped"; };
    virtual const char * getMD5() override { return "c7d23ad8985ecc1a7be1fe0399ab384b"; };

  };

}
#endif
