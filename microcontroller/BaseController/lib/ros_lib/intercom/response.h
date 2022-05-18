#ifndef _ROS_intercom_response_h
#define _ROS_intercom_response_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace intercom
{

  class response : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _text_type;
      _text_type text;

    response():
      header(),
      text("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_text = strlen(this->text);
      varToArr(outbuffer + offset, length_text);
      offset += 4;
      memcpy(outbuffer + offset, this->text, length_text);
      offset += length_text;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_text;
      arrToVar(length_text, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_text; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_text-1]=0;
      this->text = (char *)(inbuffer + offset-1);
      offset += length_text;
     return offset;
    }

    virtual const char * getType() override { return "intercom/response"; };
    virtual const char * getMD5() override { return "b925c3f72cbc309dc37196f73e57487f"; };

  };

}
#endif
