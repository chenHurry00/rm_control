//
// Created by peter on 2021/7/22.
//

#include "rm_referee/chassis_gimbal_shooter_referee.h"

namespace rm_referee
{
ChassisGimbalShooterReferee::ChassisGimbalShooterReferee(ros::NodeHandle& nh) : ChassisGimbalReferee(nh)
{
}

void ChassisGimbalShooterReferee::run()
{
  ChassisGimbalReferee::run();
}

void ChassisGimbalShooterReferee::drawUi(const ros::Time& time)
{
  ChassisGimbalReferee::drawUi(time);
  if (data_.referee_.referee_data_.robot_id_ != rm_common::RobotId::BLUE_HERO &&
      data_.referee_.referee_data_.robot_id_ != rm_common::RobotId::RED_HERO)
    trigger_change_ui_->update("target", data_.switch_detection_data_.request.target, data_.shoot_cmd_data_.burst_mode,
                               data_.switch_detection_data_.request.armor_target,
                               data_.switch_detection_data_.request.color == rm_msgs::StatusChangeRequest::RED);
  else
    trigger_change_ui_->update("target", data_.gimbal_cmd_data_.eject ? 1 : 0, data_.shoot_cmd_data_.burst_mode,
                               data_.switch_detection_data_.request.armor_target,
                               data_.switch_detection_data_.request.color == rm_msgs::StatusChangeRequest::RED);
  trigger_change_ui_->update("exposure", data_.switch_detection_data_.request.exposure, false);
  fixed_ui_->update();
}

}  // namespace rm_referee
