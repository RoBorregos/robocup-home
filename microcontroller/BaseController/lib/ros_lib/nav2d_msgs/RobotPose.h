#ifndef _ROS_nav2d_msgs_RobotPose_h
#define _ROS_nav2d_msgs_RobotPose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose2D.h"

namespace nav2d_msgs
{

  class RobotPose : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _robot_id_type;
      _robot_id_type robot_id;
      typedef geometry_msgs::Pose2D _pose_type;
      _pose_type pose;

    RobotPose():
      header(),
      robot_id(0),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_robot_id;
      u_robot_id.real = this->robot_id;
      *(outbuffer + offset + 0) = (u_robot_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_robot_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_robot_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_robot_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->robot_id);
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_robot_id;
      u_robot_id.base = 0;
      u_robot_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_robot_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_robot_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_robot_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->robot_id = u_robot_id.real;
      offset += sizeof(this->robot_id);
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "nav2d_msgs/RobotPose"; };
    virtual const char * getMD5() override { return "da85cb23bda44bed5435973e99adc0ea"; };

  };

}
#endif
