//
// Created by peter on 2021/7/22.
//

#include "rm_referee/chassis_gimbal_referee.h"

namespace rm_referee
{
ChassisGimbalReferee::ChassisGimbalReferee(ros::NodeHandle& nh) : RefereeBase(nh)
{
  ros::NodeHandle ui_nh(nh, "ui");
  trigger_change_ui_ = new TriggerChangeUi(ui_nh, data_);
  time_change_ui_ = new TimeChangeUi(ui_nh, data_);
  flash_ui_ = new FlashUi(ui_nh, data_);
  fixed_ui_ = new FixedUi(ui_nh, data_);
  gimbal_chassis_ui_ = new GimbalChassisUi(ui_nh, data_);
}

void ChassisGimbalReferee::drawUi(const ros::Time& time)
{
  RefereeBase::drawUi(time);
  time_change_ui_->update("capacitor", time);
  if (data_.dbus_data_.s_l == rm_msgs::DbusData::MID && data_.dbus_data_.s_r == rm_msgs::DbusData::UP)
  {
    trigger_change_ui_->update("chassis", data_.chassis_cmd_data_.mode, false, 1, false);
  }
  else
  {
    trigger_change_ui_->update("chassis", data_.chassis_cmd_data_.mode,
                               data_.chassis_cmd_data_.power_limit_state == rm_common::PowerLimit::BURST, 0,
                               data_.chassis_cmd_data_.power_limit_state == rm_common::PowerLimit::CHARGE);
  }
  flash_ui_->update("spin", time,
                    data_.chassis_cmd_data_.mode == rm_msgs::ChassisCmd::GYRO && data_.vel_cmd_data_.angular.z != 0.);
  flash_ui_->update("armor0", time);
  flash_ui_->update("armor1", time);
  flash_ui_->update("armor2", time);
  flash_ui_->update("armor3", time);
}

}  // namespace rm_referee
