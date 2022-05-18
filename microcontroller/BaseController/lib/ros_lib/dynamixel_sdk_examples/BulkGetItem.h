#ifndef _ROS_SERVICE_BulkGetItem_h
#define _ROS_SERVICE_BulkGetItem_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_sdk_examples
{

static const char BULKGETITEM[] = "dynamixel_sdk_examples/BulkGetItem";

  class BulkGetItemRequest : public ros::Msg
  {
    public:
      typedef uint8_t _id1_type;
      _id1_type id1;
      typedef uint8_t _id2_type;
      _id2_type id2;
      typedef const char* _item1_type;
      _item1_type item1;
      typedef const char* _item2_type;
      _item2_type item2;

    BulkGetItemRequest():
      id1(0),
      id2(0),
      item1(""),
      item2("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id1);
      *(outbuffer + offset + 0) = (this->id2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id2);
      uint32_t length_item1 = strlen(this->item1);
      varToArr(outbuffer + offset, length_item1);
      offset += 4;
      memcpy(outbuffer + offset, this->item1, length_item1);
      offset += length_item1;
      uint32_t length_item2 = strlen(this->item2);
      varToArr(outbuffer + offset, length_item2);
      offset += 4;
      memcpy(outbuffer + offset, this->item2, length_item2);
      offset += length_item2;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->id1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id1);
      this->id2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id2);
      uint32_t length_item1;
      arrToVar(length_item1, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_item1; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_item1-1]=0;
      this->item1 = (char *)(inbuffer + offset-1);
      offset += length_item1;
      uint32_t length_item2;
      arrToVar(length_item2, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_item2; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_item2-1]=0;
      this->item2 = (char *)(inbuffer + offset-1);
      offset += length_item2;
     return offset;
    }

    virtual const char * getType() override { return BULKGETITEM; };
    virtual const char * getMD5() override { return "f55042aa3aa499ad4bb778a05d278e5e"; };

  };

  class BulkGetItemResponse : public ros::Msg
  {
    public:
      typedef int32_t _value1_type;
      _value1_type value1;
      typedef int32_t _value2_type;
      _value2_type value2;

    BulkGetItemResponse():
      value1(0),
      value2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_value1;
      u_value1.real = this->value1;
      *(outbuffer + offset + 0) = (u_value1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_value1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_value1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_value1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value1);
      union {
        int32_t real;
        uint32_t base;
      } u_value2;
      u_value2.real = this->value2;
      *(outbuffer + offset + 0) = (u_value2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_value2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_value2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_value2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_value1;
      u_value1.base = 0;
      u_value1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_value1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_value1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_value1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->value1 = u_value1.real;
      offset += sizeof(this->value1);
      union {
        int32_t real;
        uint32_t base;
      } u_value2;
      u_value2.base = 0;
      u_value2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_value2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_value2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_value2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->value2 = u_value2.real;
      offset += sizeof(this->value2);
     return offset;
    }

    virtual const char * getType() override { return BULKGETITEM; };
    virtual const char * getMD5() override { return "e5963c91be0598b7f68fed70b98f2326"; };

  };

  class BulkGetItem {
    public:
    typedef BulkGetItemRequest Request;
    typedef BulkGetItemResponse Response;
  };

}
#endif
