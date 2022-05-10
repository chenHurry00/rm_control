//
// Created by chenzheng on 7/20/21.
//

#pragma once

#include "rm_referee/chassis_gimbal_shooter_referee.h"

namespace rm_referee
{
class ChassisGimbalShooterCoverReferee : public ChassisGimbalShooterReferee
{
public:
  explicit ChassisGimbalShooterCoverReferee(ros::NodeHandle& nh);
  void run() override;

protected:
  void drawUi(const ros::Time& time) override;
};
}  // namespace rm_referee
