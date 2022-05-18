#ifndef _ROS_actions_navServGoal_h
#define _ROS_actions_navServGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace actions
{

  class navServGoal : public ros::Msg
  {
    public:
      typedef const char* _target_location_type;
      _target_location_type target_location;

    navServGoal():
      target_location("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_target_location = strlen(this->target_location);
      varToArr(outbuffer + offset, length_target_location);
      offset += 4;
      memcpy(outbuffer + offset, this->target_location, length_target_location);
      offset += length_target_location;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_target_location;
      arrToVar(length_target_location, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_target_location; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_target_location-1]=0;
      this->target_location = (char *)(inbuffer + offset-1);
      offset += length_target_location;
     return offset;
    }

    virtual const char * getType() override { return "actions/navServGoal"; };
    virtual const char * getMD5() override { return "7542a4ede8f738f8eee30afbd0d32c21"; };

  };

}
#endif
