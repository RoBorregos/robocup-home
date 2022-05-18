#ifndef _ROS_SERVICE_StartStop_h
#define _ROS_SERVICE_StartStop_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosmon_msgs
{

static const char STARTSTOP[] = "rosmon_msgs/StartStop";

  class StartStopRequest : public ros::Msg
  {
    public:
      typedef const char* _node_type;
      _node_type node;
      typedef const char* _ns_type;
      _ns_type ns;
      typedef uint8_t _action_type;
      _action_type action;
      enum { START =  1 };
      enum { STOP =  2 };
      enum { RESTART =  3 };

    StartStopRequest():
      node(""),
      ns(""),
      action(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_node = strlen(this->node);
      varToArr(outbuffer + offset, length_node);
      offset += 4;
      memcpy(outbuffer + offset, this->node, length_node);
      offset += length_node;
      uint32_t length_ns = strlen(this->ns);
      varToArr(outbuffer + offset, length_ns);
      offset += 4;
      memcpy(outbuffer + offset, this->ns, length_ns);
      offset += length_ns;
      *(outbuffer + offset + 0) = (this->action >> (8 * 0)) & 0xFF;
      offset += sizeof(this->action);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_node;
      arrToVar(length_node, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_node; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_node-1]=0;
      this->node = (char *)(inbuffer + offset-1);
      offset += length_node;
      uint32_t length_ns;
      arrToVar(length_ns, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ns; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ns-1]=0;
      this->ns = (char *)(inbuffer + offset-1);
      offset += length_ns;
      this->action =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->action);
     return offset;
    }

    virtual const char * getType() override { return STARTSTOP; };
    virtual const char * getMD5() override { return "a95e7883b3762847035c73ffc86de3ea"; };

  };

  class StartStopResponse : public ros::Msg
  {
    public:

    StartStopResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return STARTSTOP; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class StartStop {
    public:
    typedef StartStopRequest Request;
    typedef StartStopResponse Response;
  };

}
#endif
