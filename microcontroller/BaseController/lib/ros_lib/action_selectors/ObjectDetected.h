#ifndef _ROS_action_selectors_ObjectDetected_h
#define _ROS_action_selectors_ObjectDetected_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace action_selectors
{

  class ObjectDetected : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef float _score_type;
      _score_type score;
      typedef float _y_min_type;
      _y_min_type y_min;
      typedef float _x_min_type;
      _x_min_type x_min;
      typedef float _y_max_type;
      _y_max_type y_max;
      typedef float _x_max_type;
      _x_max_type x_max;

    ObjectDetected():
      name(""),
      score(0),
      y_min(0),
      x_min(0),
      y_max(0),
      x_max(0)
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
      union {
        float real;
        uint32_t base;
      } u_score;
      u_score.real = this->score;
      *(outbuffer + offset + 0) = (u_score.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_score.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_score.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_score.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->score);
      union {
        float real;
        uint32_t base;
      } u_y_min;
      u_y_min.real = this->y_min;
      *(outbuffer + offset + 0) = (u_y_min.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_min.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_min.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_min.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_min);
      union {
        float real;
        uint32_t base;
      } u_x_min;
      u_x_min.real = this->x_min;
      *(outbuffer + offset + 0) = (u_x_min.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_min.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_min.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_min.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_min);
      union {
        float real;
        uint32_t base;
      } u_y_max;
      u_y_max.real = this->y_max;
      *(outbuffer + offset + 0) = (u_y_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_max.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_max);
      union {
        float real;
        uint32_t base;
      } u_x_max;
      u_x_max.real = this->x_max;
      *(outbuffer + offset + 0) = (u_x_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_max.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_max);
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
      union {
        float real;
        uint32_t base;
      } u_score;
      u_score.base = 0;
      u_score.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_score.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_score.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_score.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->score = u_score.real;
      offset += sizeof(this->score);
      union {
        float real;
        uint32_t base;
      } u_y_min;
      u_y_min.base = 0;
      u_y_min.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_min.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_min.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_min.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y_min = u_y_min.real;
      offset += sizeof(this->y_min);
      union {
        float real;
        uint32_t base;
      } u_x_min;
      u_x_min.base = 0;
      u_x_min.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_min.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_min.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_min.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_min = u_x_min.real;
      offset += sizeof(this->x_min);
      union {
        float real;
        uint32_t base;
      } u_y_max;
      u_y_max.base = 0;
      u_y_max.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_max.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_max.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_max.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y_max = u_y_max.real;
      offset += sizeof(this->y_max);
      union {
        float real;
        uint32_t base;
      } u_x_max;
      u_x_max.base = 0;
      u_x_max.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_max.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_max.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_max.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_max = u_x_max.real;
      offset += sizeof(this->x_max);
     return offset;
    }

    virtual const char * getType() override { return "action_selectors/ObjectDetected"; };
    virtual const char * getMD5() override { return "fe6f5df342e183602506c829bc49c703"; };

  };

}
#endif
