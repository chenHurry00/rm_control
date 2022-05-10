//
// Created by peter on 2021/7/22.
//

#include "rm_referee/chassis_gimbal_shooter_cover_referee.h"

namespace rm_referee
{
ChassisGimbalShooterCoverReferee::ChassisGimbalShooterCoverReferee(ros::NodeHandle& nh)
  : ChassisGimbalShooterReferee(nh)
{
}

void ChassisGimbalShooterCoverReferee::run()
{
  ChassisGimbalShooterReferee::run();
}

void ChassisGimbalShooterCoverReferee::drawUi(const ros::Time& time)
{
  ChassisGimbalShooterReferee::drawUi(time);
  flash_ui_->update("cover", time, !data_.cover_cmd_data_.data);
}

};  // namespace rm_referee
