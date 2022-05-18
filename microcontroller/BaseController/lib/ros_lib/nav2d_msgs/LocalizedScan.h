#ifndef _ROS_nav2d_msgs_LocalizedScan_h
#define _ROS_nav2d_msgs_LocalizedScan_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/LaserScan.h"

namespace nav2d_msgs
{

  class LocalizedScan : public ros::Msg
  {
    public:
      typedef int8_t _robot_id_type;
      _robot_id_type robot_id;
      typedef int8_t _laser_type_type;
      _laser_type_type laser_type;
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _yaw_type;
      _yaw_type yaw;
      typedef sensor_msgs::LaserScan _scan_type;
      _scan_type scan;

    LocalizedScan():
      robot_id(0),
      laser_type(0),
      x(0),
      y(0),
      yaw(0),
      scan()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_robot_id;
      u_robot_id.real = this->robot_id;
      *(outbuffer + offset + 0) = (u_robot_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->robot_id);
      union {
        int8_t real;
        uint8_t base;
      } u_laser_type;
      u_laser_type.real = this->laser_type;
      *(outbuffer + offset + 0) = (u_laser_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->laser_type);
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.real = this->yaw;
      *(outbuffer + offset + 0) = (u_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw);
      offset += this->scan.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_robot_id;
      u_robot_id.base = 0;
      u_robot_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->robot_id = u_robot_id.real;
      offset += sizeof(this->robot_id);
      union {
        int8_t real;
        uint8_t base;
      } u_laser_type;
      u_laser_type.base = 0;
      u_laser_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->laser_type = u_laser_type.real;
      offset += sizeof(this->laser_type);
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.base = 0;
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw = u_yaw.real;
      offset += sizeof(this->yaw);
      offset += this->scan.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "nav2d_msgs/LocalizedScan"; };
    virtual const char * getMD5() override { return "bab53504723a56692b3864ccf3dfe635"; };

  };

}
#endif
