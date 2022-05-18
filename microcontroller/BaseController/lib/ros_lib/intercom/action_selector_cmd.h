#ifndef _ROS_intercom_action_selector_cmd_h
#define _ROS_intercom_action_selector_cmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace intercom
{

  class action_selector_cmd : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _intent_type;
      _intent_type intent;
      typedef const char* _action_client_binded_type;
      _action_client_binded_type action_client_binded;
      typedef const char* _args_type;
      _args_type args;

    action_selector_cmd():
      header(),
      intent(""),
      action_client_binded(""),
      args("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_intent = strlen(this->intent);
      varToArr(outbuffer + offset, length_intent);
      offset += 4;
      memcpy(outbuffer + offset, this->intent, length_intent);
      offset += length_intent;
      uint32_t length_action_client_binded = strlen(this->action_client_binded);
      varToArr(outbuffer + offset, length_action_client_binded);
      offset += 4;
      memcpy(outbuffer + offset, this->action_client_binded, length_action_client_binded);
      offset += length_action_client_binded;
      uint32_t length_args = strlen(this->args);
      varToArr(outbuffer + offset, length_args);
      offset += 4;
      memcpy(outbuffer + offset, this->args, length_args);
      offset += length_args;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_intent;
      arrToVar(length_intent, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_intent; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_intent-1]=0;
      this->intent = (char *)(inbuffer + offset-1);
      offset += length_intent;
      uint32_t length_action_client_binded;
      arrToVar(length_action_client_binded, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_action_client_binded; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_action_client_binded-1]=0;
      this->action_client_binded = (char *)(inbuffer + offset-1);
      offset += length_action_client_binded;
      uint32_t length_args;
      arrToVar(length_args, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_args; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_args-1]=0;
      this->args = (char *)(inbuffer + offset-1);
      offset += length_args;
     return offset;
    }

    virtual const char * getType() override { return "intercom/action_selector_cmd"; };
    virtual const char * getMD5() override { return "c6d6c1c3b3f584722027228a8f806439"; };

  };

}
#endif
