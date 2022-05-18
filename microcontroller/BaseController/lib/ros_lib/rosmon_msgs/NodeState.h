#ifndef _ROS_rosmon_msgs_NodeState_h
#define _ROS_rosmon_msgs_NodeState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosmon_msgs
{

  class NodeState : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _ns_type;
      _ns_type ns;
      typedef uint8_t _state_type;
      _state_type state;
      typedef uint32_t _restart_count_type;
      _restart_count_type restart_count;
      typedef float _user_load_type;
      _user_load_type user_load;
      typedef float _system_load_type;
      _system_load_type system_load;
      typedef uint64_t _memory_type;
      _memory_type memory;
      enum { IDLE =  0      };
      enum { RUNNING =  1   };
      enum { CRASHED =  2   };
      enum { WAITING =  3   };

    NodeState():
      name(""),
      ns(""),
      state(0),
      restart_count(0),
      user_load(0),
      system_load(0),
      memory(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_ns = strlen(this->ns);
      varToArr(outbuffer + offset, length_ns);
      offset += 4;
      memcpy(outbuffer + offset, this->ns, length_ns);
      offset += length_ns;
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      *(outbuffer + offset + 0) = (this->restart_count >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->restart_count >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->restart_count >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->restart_count >> (8 * 3)) & 0xFF;
      offset += sizeof(this->restart_count);
      union {
        float real;
        uint32_t base;
      } u_user_load;
      u_user_load.real = this->user_load;
      *(outbuffer + offset + 0) = (u_user_load.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_user_load.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_user_load.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_user_load.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->user_load);
      union {
        float real;
        uint32_t base;
      } u_system_load;
      u_system_load.real = this->system_load;
      *(outbuffer + offset + 0) = (u_system_load.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_system_load.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_system_load.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_system_load.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->system_load);
      *(outbuffer + offset + 0) = (this->memory >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->memory >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->memory >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->memory >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->memory >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->memory >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->memory >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->memory >> (8 * 7)) & 0xFF;
      offset += sizeof(this->memory);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_ns;
      arrToVar(length_ns, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ns; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ns-1]=0;
      this->ns = (char *)(inbuffer + offset-1);
      offset += length_ns;
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
      this->restart_count =  ((uint32_t) (*(inbuffer + offset)));
      this->restart_count |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->restart_count |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->restart_count |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->restart_count);
      union {
        float real;
        uint32_t base;
      } u_user_load;
      u_user_load.base = 0;
      u_user_load.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_user_load.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_user_load.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_user_load.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->user_load = u_user_load.real;
      offset += sizeof(this->user_load);
      union {
        float real;
        uint32_t base;
      } u_system_load;
      u_system_load.base = 0;
      u_system_load.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_system_load.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_system_load.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_system_load.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->system_load = u_system_load.real;
      offset += sizeof(this->system_load);
      this->memory =  ((uint64_t) (*(inbuffer + offset)));
      this->memory |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->memory |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->memory |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->memory |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->memory |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->memory |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->memory |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->memory);
     return offset;
    }

    virtual const char * getType() override { return "rosmon_msgs/NodeState"; };
    virtual const char * getMD5() override { return "87292cdbe5a500a95e32714adf10e6dd"; };

  };

}
#endif
