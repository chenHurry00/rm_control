//
// Created by peter on 2020/12/3.
//

#pragma once

#include "rm_referee/common/data.h"
#include "rm_referee/referee/ui.h"

#include <iostream>
#include <queue>
#include <tf/transform_listener.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_manager.h>
#include <controller_manager_msgs/SwitchController.h>

namespace rm_referee
{
class RefereeBase
{
public:
  explicit RefereeBase(ros::NodeHandle& nh);
  enum
  {
    PASSIVE,
    IDLE,
    RC,
    PC
  };
  virtual void run();

protected:
  virtual void drawUi(const ros::Time& time)
  {
    data_.referee_.sendUi(time);
  }

  Data data_;
  ros::NodeHandle nh_;

  bool remote_is_open_{};
  int state_ = PASSIVE;
};

}  // namespace rm_referee
