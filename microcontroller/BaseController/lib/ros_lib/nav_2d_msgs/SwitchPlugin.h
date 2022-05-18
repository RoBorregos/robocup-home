#ifndef _ROS_SERVICE_SwitchPlugin_h
#define _ROS_SERVICE_SwitchPlugin_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nav_2d_msgs
{

static const char SWITCHPLUGIN[] = "nav_2d_msgs/SwitchPlugin";

  class SwitchPluginRequest : public ros::Msg
  {
    public:
      typedef const char* _new_plugin_type;
      _new_plugin_type new_plugin;

    SwitchPluginRequest():
      new_plugin("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_new_plugin = strlen(this->new_plugin);
      varToArr(outbuffer + offset, length_new_plugin);
      offset += 4;
      memcpy(outbuffer + offset, this->new_plugin, length_new_plugin);
      offset += length_new_plugin;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_new_plugin;
      arrToVar(length_new_plugin, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_new_plugin; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_new_plugin-1]=0;
      this->new_plugin = (char *)(inbuffer + offset-1);
      offset += length_new_plugin;
     return offset;
    }

    virtual const char * getType() override { return SWITCHPLUGIN; };
    virtual const char * getMD5() override { return "d89ec9b6d29038f89e1e1ed70f9eeeeb"; };

  };

  class SwitchPluginResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;

    SwitchPluginResponse():
      success(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
     return offset;
    }

    virtual const char * getType() override { return SWITCHPLUGIN; };
    virtual const char * getMD5() override { return "937c9679a518e3a18d831e57125ea522"; };

  };

  class SwitchPlugin {
    public:
    typedef SwitchPluginRequest Request;
    typedef SwitchPluginResponse Response;
  };

}
#endif
