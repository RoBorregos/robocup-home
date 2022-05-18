#ifndef _ROS_nav_2d_msgs_Polygon2D_h
#define _ROS_nav_2d_msgs_Polygon2D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_2d_msgs/Point2D.h"

namespace nav_2d_msgs
{

  class Polygon2D : public ros::Msg
  {
    public:
      uint32_t points_length;
      typedef nav_2d_msgs::Point2D _points_type;
      _points_type st_points;
      _points_type * points;

    Polygon2D():
      points_length(0), st_points(), points(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->points_length);
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->points_length);
      if(points_lengthT > points_length)
        this->points = (nav_2d_msgs::Point2D*)realloc(this->points, points_lengthT * sizeof(nav_2d_msgs::Point2D));
      points_length = points_lengthT;
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(nav_2d_msgs::Point2D));
      }
     return offset;
    }

    virtual const char * getType() override { return "nav_2d_msgs/Polygon2D"; };
    virtual const char * getMD5() override { return "8f02263beef99aa03117a577a3eb879d"; };

  };

}
#endif
