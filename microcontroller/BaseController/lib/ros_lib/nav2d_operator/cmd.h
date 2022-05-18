#ifndef _ROS_nav2d_operator_cmd_h
#define _ROS_nav2d_operator_cmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nav2d_operator
{

  class cmd : public ros::Msg
  {
    public:
      typedef float _Velocity_type;
      _Velocity_type Velocity;
      typedef float _Turn_type;
      _Turn_type Turn;
      typedef int8_t _Mode_type;
      _Mode_type Mode;

    cmd():
      Velocity(0),
      Turn(0),
      Mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->Velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->Turn);
      union {
        int8_t real;
        uint8_t base;
      } u_Mode;
      u_Mode.real = this->Mode;
      *(outbuffer + offset + 0) = (u_Mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Turn));
      union {
        int8_t real;
        uint8_t base;
      } u_Mode;
      u_Mode.base = 0;
      u_Mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Mode = u_Mode.real;
      offset += sizeof(this->Mode);
     return offset;
    }

    virtual const char * getType() override { return "nav2d_operator/cmd"; };
    virtual const char * getMD5() override { return "90c9a043660646e2102f124332ecb8b7"; };

  };

}
#endif
