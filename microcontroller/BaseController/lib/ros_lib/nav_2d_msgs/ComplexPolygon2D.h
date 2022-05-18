#ifndef _ROS_nav_2d_msgs_ComplexPolygon2D_h
#define _ROS_nav_2d_msgs_ComplexPolygon2D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_2d_msgs/Polygon2D.h"

namespace nav_2d_msgs
{

  class ComplexPolygon2D : public ros::Msg
  {
    public:
      typedef nav_2d_msgs::Polygon2D _outer_type;
      _outer_type outer;
      uint32_t inner_length;
      typedef nav_2d_msgs::Polygon2D _inner_type;
      _inner_type st_inner;
      _inner_type * inner;

    ComplexPolygon2D():
      outer(),
      inner_length(0), st_inner(), inner(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->outer.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->inner_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->inner_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->inner_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->inner_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->inner_length);
      for( uint32_t i = 0; i < inner_length; i++){
      offset += this->inner[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->outer.deserialize(inbuffer + offset);
      uint32_t inner_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      inner_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      inner_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      inner_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->inner_length);
      if(inner_lengthT > inner_length)
        this->inner = (nav_2d_msgs::Polygon2D*)realloc(this->inner, inner_lengthT * sizeof(nav_2d_msgs::Polygon2D));
      inner_length = inner_lengthT;
      for( uint32_t i = 0; i < inner_length; i++){
      offset += this->st_inner.deserialize(inbuffer + offset);
        memcpy( &(this->inner[i]), &(this->st_inner), sizeof(nav_2d_msgs::Polygon2D));
      }
     return offset;
    }

    virtual const char * getType() override { return "nav_2d_msgs/ComplexPolygon2D"; };
    virtual const char * getMD5() override { return "f93841a14f7caf40d600ff8c62446616"; };

  };

}
#endif
