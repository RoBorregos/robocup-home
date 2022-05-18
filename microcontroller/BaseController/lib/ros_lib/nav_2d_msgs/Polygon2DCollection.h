#ifndef _ROS_nav_2d_msgs_Polygon2DCollection_h
#define _ROS_nav_2d_msgs_Polygon2DCollection_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "nav_2d_msgs/ComplexPolygon2D.h"
#include "std_msgs/ColorRGBA.h"

namespace nav_2d_msgs
{

  class Polygon2DCollection : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t polygons_length;
      typedef nav_2d_msgs::ComplexPolygon2D _polygons_type;
      _polygons_type st_polygons;
      _polygons_type * polygons;
      uint32_t colors_length;
      typedef std_msgs::ColorRGBA _colors_type;
      _colors_type st_colors;
      _colors_type * colors;

    Polygon2DCollection():
      header(),
      polygons_length(0), st_polygons(), polygons(nullptr),
      colors_length(0), st_colors(), colors(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->polygons_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->polygons_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->polygons_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->polygons_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->polygons_length);
      for( uint32_t i = 0; i < polygons_length; i++){
      offset += this->polygons[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->colors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->colors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->colors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->colors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->colors_length);
      for( uint32_t i = 0; i < colors_length; i++){
      offset += this->colors[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t polygons_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      polygons_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      polygons_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      polygons_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->polygons_length);
      if(polygons_lengthT > polygons_length)
        this->polygons = (nav_2d_msgs::ComplexPolygon2D*)realloc(this->polygons, polygons_lengthT * sizeof(nav_2d_msgs::ComplexPolygon2D));
      polygons_length = polygons_lengthT;
      for( uint32_t i = 0; i < polygons_length; i++){
      offset += this->st_polygons.deserialize(inbuffer + offset);
        memcpy( &(this->polygons[i]), &(this->st_polygons), sizeof(nav_2d_msgs::ComplexPolygon2D));
      }
      uint32_t colors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      colors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      colors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      colors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->colors_length);
      if(colors_lengthT > colors_length)
        this->colors = (std_msgs::ColorRGBA*)realloc(this->colors, colors_lengthT * sizeof(std_msgs::ColorRGBA));
      colors_length = colors_lengthT;
      for( uint32_t i = 0; i < colors_length; i++){
      offset += this->st_colors.deserialize(inbuffer + offset);
        memcpy( &(this->colors[i]), &(this->st_colors), sizeof(std_msgs::ColorRGBA));
      }
     return offset;
    }

    virtual const char * getType() override { return "nav_2d_msgs/Polygon2DCollection"; };
    virtual const char * getMD5() override { return "a0186b831cfbcfeafd72a58884a4efe9"; };

  };

}
#endif
