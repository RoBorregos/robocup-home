#ifndef _ROS_action_selectors_RawInput_h
#define _ROS_action_selectors_RawInput_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace action_selectors
{

  class RawInput : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _isWoman_type;
      _isWoman_type isWoman;
      typedef const char* _inputText_type;
      _inputText_type inputText;

    RawInput():
      header(),
      isWoman(0),
      inputText("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_isWoman;
      u_isWoman.real = this->isWoman;
      *(outbuffer + offset + 0) = (u_isWoman.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isWoman);
      uint32_t length_inputText = strlen(this->inputText);
      varToArr(outbuffer + offset, length_inputText);
      offset += 4;
      memcpy(outbuffer + offset, this->inputText, length_inputText);
      offset += length_inputText;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_isWoman;
      u_isWoman.base = 0;
      u_isWoman.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isWoman = u_isWoman.real;
      offset += sizeof(this->isWoman);
      uint32_t length_inputText;
      arrToVar(length_inputText, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_inputText; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_inputText-1]=0;
      this->inputText = (char *)(inbuffer + offset-1);
      offset += length_inputText;
     return offset;
    }

    virtual const char * getType() override { return "action_selectors/RawInput"; };
    virtual const char * getMD5() override { return "581b5ef4709397dc37affb30ae6be65a"; };

  };

}
#endif
