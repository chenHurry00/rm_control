//
// Created by ljq on 2022/5/17.
//
#include "rm_referee/engineer_referee.h"

namespace rm_referee
{
EngineerReferee::EngineerReferee(ros::NodeHandle& nh, Data& data) : RobotReferee(nh, data)
{
  EngineerReferee::joint_state_sub_ =
      nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &EngineerReferee::jointStateCallback, this);
  EngineerReferee::manual_data_sub_ =
      nh.subscribe<rm_msgs::ManualToReferee>("/manual_to_referee", 10, &EngineerReferee::manualDataCallBack, this);
  interactive_data_sender_ = new Graph(data.base_);
}

void EngineerReferee::run()
{
  RobotReferee::run();
}

void EngineerReferee::interactiveDataCallBack(const rm_referee::InteractiveData& interactive_data_,
                                              const ros::Time& last_get_)
{
  RefereeBase::interactiveDataCallBack(interactive_data_, last_get_);
  //  if (interactive_data_.header_data_.data_cmd_id_ == 0x0201 && interactive_data_.data_ != sentry_mode_)
  //    interactive_data_sender_->sendInteractiveData(
  //        0x0200, data_.base_.robot_color_ == "blue" ? rm_referee::RobotId::BLUE_SENTRY : rm_referee::RED_SENTRY,
  //        sentry_mode_);
}
void EngineerReferee::jointStateCallback(const sensor_msgs::JointState::ConstPtr& data)
{
  RefereeBase::jointStateCallback(data);
  time_change_ui_->update("effort", ros::Time::now());
  if (!data_.joint_state_.name.empty())
    flash_ui_->update("card_warning", ros::Time::now(), data_.joint_state_.effort[0] < 1.5);
  //    trigger_change_ui_->update("jog", jog_joint_name);
}

void EngineerReferee::actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data)
{
  RefereeBase::actuatorStateCallback(data);
  time_change_ui_->update("temperature", ros::Time::now());
}

void EngineerReferee::cardCmdDataCallback(const rm_msgs::StateCmd::ConstPtr& data)
{
  RefereeBase::cardCmdDataCallback(data);
  trigger_change_ui_->update("card", 0, data_.card_cmd_data_.mode);
}

void EngineerReferee::engineerCmdDataCallback(const rm_msgs::EngineerCmd ::ConstPtr& data)
{
  RefereeBase::engineerCmdDataCallback(data);
  if (data_.engineer_cmd_data_.symbol != symbol)
  {
    if (data_.engineer_cmd_data_.total_steps != 0)
      time_change_ui_->update("progress", ros::Time::now(),
                              data_.engineer_cmd_data_.finished_step / data_.engineer_cmd_data_.total_steps);
    else
      time_change_ui_->update("progress", ros::Time::now(), 0.);
  }
  symbol = data_.engineer_cmd_data_.symbol;
  trigger_change_ui_->update("step", data_.engineer_cmd_data_.step_queue_name);
}

void EngineerReferee::manualDataCallBack(const rm_msgs::ManualToReferee::ConstPtr& data)
{
  RefereeBase::manualDataCallBack(data);
  flash_ui_->update("calibration", ros::Time::now(), data_.manual_to_referee_data_.engineer_calibration_state);
}

}  // namespace rm_referee
