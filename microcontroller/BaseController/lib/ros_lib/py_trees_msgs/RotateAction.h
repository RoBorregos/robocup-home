#ifndef _ROS_py_trees_msgs_RotateAction_h
#define _ROS_py_trees_msgs_RotateAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "py_trees_msgs/RotateActionGoal.h"
#include "py_trees_msgs/RotateActionResult.h"
#include "py_trees_msgs/RotateActionFeedback.h"

namespace py_trees_msgs
{

  class RotateAction : public ros::Msg
  {
    public:
      typedef py_trees_msgs::RotateActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef py_trees_msgs::RotateActionResult _action_result_type;
      _action_result_type action_result;
      typedef py_trees_msgs::RotateActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    RotateAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "py_trees_msgs/RotateAction"; };
    virtual const char * getMD5() override { return "c68d9e0a719660a32868a081a56942db"; };

  };

}
#endif
