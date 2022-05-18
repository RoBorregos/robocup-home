#ifndef _ROS_nav_2d_msgs_Pose2DStamped_h
#define _ROS_nav_2d_msgs_Pose2DStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose2D.h"

namespace nav_2d_msgs
{

  class Pose2DStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose2D _pose_type;
      _pose_type pose;

    Pose2DStamped():
      header(),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "nav_2d_msgs/Pose2DStamped"; };
    virtual const char * getMD5() override { return "b5f1e28823201bc5ea7e310fc49d253f"; };

  };

}
#endif
