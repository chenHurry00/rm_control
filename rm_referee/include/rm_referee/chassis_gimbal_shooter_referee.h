//
// Created by qiayuan on 5/22/21.
//

#pragma once

#include "rm_referee/chassis_gimbal_referee.h"
#include <rm_common/decision/calibration_queue.h>

namespace rm_referee
{
class ChassisGimbalShooterReferee : public ChassisGimbalReferee
{
public:
  explicit ChassisGimbalShooterReferee(ros::NodeHandle& nh);
  void run() override;

protected:
  void drawUi(const ros::Time& time) override;
};
}  // namespace rm_referee
