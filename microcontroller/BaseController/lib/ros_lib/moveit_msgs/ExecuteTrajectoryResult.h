#ifndef _ROS_moveit_msgs_ExecuteTrajectoryResult_h
#define _ROS_moveit_msgs_ExecuteTrajectoryResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moveit_msgs/MoveItErrorCodes.h"

namespace moveit_msgs
{

  class ExecuteTrajectoryResult : public ros::Msg
  {
    public:
      typedef moveit_msgs::MoveItErrorCodes _error_code_type;
      _error_code_type error_code;

    ExecuteTrajectoryResult():
      error_code()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->error_code.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->error_code.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "moveit_msgs/ExecuteTrajectoryResult"; };
    virtual const char * getMD5() override { return "a367304b29bf35b99db616894f470bab"; };

  };

}
#endif
