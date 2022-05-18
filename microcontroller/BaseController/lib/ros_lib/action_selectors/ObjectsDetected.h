#ifndef _ROS_action_selectors_ObjectsDetected_h
#define _ROS_action_selectors_ObjectsDetected_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "action_selectors/ObjectDetected.h"

namespace action_selectors
{

  class ObjectsDetected : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t objects_detected_length;
      typedef action_selectors::ObjectDetected _objects_detected_type;
      _objects_detected_type st_objects_detected;
      _objects_detected_type * objects_detected;

    ObjectsDetected():
      header(),
      objects_detected_length(0), st_objects_detected(), objects_detected(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->objects_detected_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->objects_detected_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->objects_detected_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->objects_detected_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->objects_detected_length);
      for( uint32_t i = 0; i < objects_detected_length; i++){
      offset += this->objects_detected[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t objects_detected_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      objects_detected_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      objects_detected_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      objects_detected_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->objects_detected_length);
      if(objects_detected_lengthT > objects_detected_length)
        this->objects_detected = (action_selectors::ObjectDetected*)realloc(this->objects_detected, objects_detected_lengthT * sizeof(action_selectors::ObjectDetected));
      objects_detected_length = objects_detected_lengthT;
      for( uint32_t i = 0; i < objects_detected_length; i++){
      offset += this->st_objects_detected.deserialize(inbuffer + offset);
        memcpy( &(this->objects_detected[i]), &(this->st_objects_detected), sizeof(action_selectors::ObjectDetected));
      }
     return offset;
    }

    virtual const char * getType() override { return "action_selectors/ObjectsDetected"; };
    virtual const char * getMD5() override { return "5cd82ac93fbfc8f20af8e12475398b74"; };

  };

}
#endif
