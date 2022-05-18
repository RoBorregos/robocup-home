#ifndef _ROS_dynamixel_sdk_examples_SyncSetPosition_h
#define _ROS_dynamixel_sdk_examples_SyncSetPosition_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_sdk_examples
{

  class SyncSetPosition : public ros::Msg
  {
    public:
      typedef uint8_t _id1_type;
      _id1_type id1;
      typedef uint8_t _id2_type;
      _id2_type id2;
      typedef int32_t _position1_type;
      _position1_type position1;
      typedef int32_t _position2_type;
      _position2_type position2;

    SyncSetPosition():
      id1(0),
      id2(0),
      position1(0),
      position2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id1);
      *(outbuffer + offset + 0) = (this->id2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id2);
      union {
        int32_t real;
        uint32_t base;
      } u_position1;
      u_position1.real = this->position1;
      *(outbuffer + offset + 0) = (u_position1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position1);
      union {
        int32_t real;
        uint32_t base;
      } u_position2;
      u_position2.real = this->position2;
      *(outbuffer + offset + 0) = (u_position2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->id1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id1);
      this->id2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id2);
      union {
        int32_t real;
        uint32_t base;
      } u_position1;
      u_position1.base = 0;
      u_position1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position1 = u_position1.real;
      offset += sizeof(this->position1);
      union {
        int32_t real;
        uint32_t base;
      } u_position2;
      u_position2.base = 0;
      u_position2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position2 = u_position2.real;
      offset += sizeof(this->position2);
     return offset;
    }

    virtual const char * getType() override { return "dynamixel_sdk_examples/SyncSetPosition"; };
    virtual const char * getMD5() override { return "c9220e34b4c74855ca69205046aec62f"; };

  };

}
#endif
