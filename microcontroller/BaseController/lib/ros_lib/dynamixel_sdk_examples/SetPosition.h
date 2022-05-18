#ifndef _ROS_dynamixel_sdk_examples_SetPosition_h
#define _ROS_dynamixel_sdk_examples_SetPosition_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_sdk_examples
{

  class SetPosition : public ros::Msg
  {
    public:
      typedef uint8_t _id_type;
      _id_type id;
      typedef int32_t _position_type;
      _position_type position;

    SetPosition():
      id(0),
      position(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_position;
      u_position.real = this->position;
      *(outbuffer + offset + 0) = (u_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_position;
      u_position.base = 0;
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position = u_position.real;
      offset += sizeof(this->position);
     return offset;
    }

    virtual const char * getType() override { return "dynamixel_sdk_examples/SetPosition"; };
    virtual const char * getMD5() override { return "0a775458729eb3272bc88b4f5f764cc8"; };

  };

}
#endif
