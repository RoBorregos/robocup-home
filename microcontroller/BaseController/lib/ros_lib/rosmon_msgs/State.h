#ifndef _ROS_rosmon_msgs_State_h
#define _ROS_rosmon_msgs_State_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "rosmon_msgs/NodeState.h"

namespace rosmon_msgs
{

  class State : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t nodes_length;
      typedef rosmon_msgs::NodeState _nodes_type;
      _nodes_type st_nodes;
      _nodes_type * nodes;

    State():
      header(),
      nodes_length(0), st_nodes(), nodes(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->nodes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nodes_length);
      for( uint32_t i = 0; i < nodes_length; i++){
      offset += this->nodes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t nodes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nodes_length);
      if(nodes_lengthT > nodes_length)
        this->nodes = (rosmon_msgs::NodeState*)realloc(this->nodes, nodes_lengthT * sizeof(rosmon_msgs::NodeState));
      nodes_length = nodes_lengthT;
      for( uint32_t i = 0; i < nodes_length; i++){
      offset += this->st_nodes.deserialize(inbuffer + offset);
        memcpy( &(this->nodes[i]), &(this->st_nodes), sizeof(rosmon_msgs::NodeState));
      }
     return offset;
    }

    virtual const char * getType() override { return "rosmon_msgs/State"; };
    virtual const char * getMD5() override { return "93fe347b2ffed78bbacee05fd2655b2e"; };

  };

}
#endif
