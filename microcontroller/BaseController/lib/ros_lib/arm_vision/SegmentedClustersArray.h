#ifndef _ROS_arm_vision_SegmentedClustersArray_h
#define _ROS_arm_vision_SegmentedClustersArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/PointCloud2.h"

namespace arm_vision
{

  class SegmentedClustersArray : public ros::Msg
  {
    public:
      uint32_t clusters_length;
      typedef sensor_msgs::PointCloud2 _clusters_type;
      _clusters_type st_clusters;
      _clusters_type * clusters;

    SegmentedClustersArray():
      clusters_length(0), st_clusters(), clusters(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->clusters_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->clusters_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->clusters_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->clusters_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clusters_length);
      for( uint32_t i = 0; i < clusters_length; i++){
      offset += this->clusters[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t clusters_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      clusters_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      clusters_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      clusters_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->clusters_length);
      if(clusters_lengthT > clusters_length)
        this->clusters = (sensor_msgs::PointCloud2*)realloc(this->clusters, clusters_lengthT * sizeof(sensor_msgs::PointCloud2));
      clusters_length = clusters_lengthT;
      for( uint32_t i = 0; i < clusters_length; i++){
      offset += this->st_clusters.deserialize(inbuffer + offset);
        memcpy( &(this->clusters[i]), &(this->st_clusters), sizeof(sensor_msgs::PointCloud2));
      }
     return offset;
    }

    virtual const char * getType() override { return "arm_vision/SegmentedClustersArray"; };
    virtual const char * getMD5() override { return "f94e5e56fc7ff419809a40b3ff371b4c"; };

  };

}
#endif
