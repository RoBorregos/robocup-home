#ifndef _ROS_actions_navServAction_h
#define _ROS_actions_navServAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "actions/navServActionGoal.h"
#include "actions/navServActionResult.h"
#include "actions/navServActionFeedback.h"

namespace actions
{

  class navServAction : public ros::Msg
  {
    public:
      typedef actions::navServActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef actions::navServActionResult _action_result_type;
      _action_result_type action_result;
      typedef actions::navServActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    navServAction():
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

    virtual const char * getType() override { return "actions/navServAction"; };
    virtual const char * getMD5() override { return "cf133f1aa6358482a1152dc3167050a1"; };

  };

}
#endif
