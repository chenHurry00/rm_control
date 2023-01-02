//
// Created by llljjjqqq on 22-11-4.
//

#include "rm_referee/ui/trigger_change_ui.h"

namespace rm_referee
{
void TriggerChangeUi::setContent(const std::string& content)
{
  graph_->setContent(content);
  display();
}

void TriggerChangeUi::display()
{
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->display();
  graph_->sendUi(ros::Time::now());
}

void ChassisTriggerChangeUi::display()
{
  if (s_l_ == rm_msgs::DbusData::MID && s_r_ == rm_msgs::DbusData::UP)
    updateConfig(chassis_mode_, false, 1, false);
  else
    updateConfig(chassis_mode_, power_limit_state_ == rm_common::PowerLimit::BURST, 0,
                 power_limit_state_ == rm_common::PowerLimit::CHARGE);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->displayTwice(true);
  graph_->sendUi(ros::Time::now());
}

void ChassisTriggerChangeUi::displayInCapacity()
{
  if (key_ctrl_ && key_shift_ && key_b_ && base_.robot_id_ != rm_referee::RobotId::RED_ENGINEER &&
      base_.robot_id_ != rm_referee::RobotId::BLUE_ENGINEER)
    updateConfig(254, 0);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->displayTwice(true);
  graph_->sendUi(ros::Time::now());
}

void ChassisTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
{
  if (main_mode == 254)
  {
    graph_->setContent("Cap reset");
    graph_->setColor(rm_referee::GraphColor::YELLOW);
    return;
  }
  graph_->setContent(getChassisState(main_mode));
  if (main_flag)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
  else if (sub_flag)
    graph_->setColor(rm_referee::GraphColor::GREEN);
  else if (sub_mode == 1)
    graph_->setColor(rm_referee::GraphColor::PINK);
  else
    graph_->setColor(rm_referee::GraphColor::WHITE);
}

std::string ChassisTriggerChangeUi::getChassisState(uint8_t mode)
{
  if (mode == rm_msgs::ChassisCmd::RAW)
    return "raw";
  else if (mode == rm_msgs::ChassisCmd::FOLLOW)
    return "follow";
  else if (mode == rm_msgs::ChassisCmd::GYRO)
    return "gyro";
  else if (mode == rm_msgs::ChassisCmd::TWIST)
    return "twist";
  else
    return "error";
}

void ChassisTriggerChangeUi::updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data)
{
  chassis_mode_ = data->mode;
  display();
}

void ChassisTriggerChangeUi::updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data)
{
  power_limit_state_ = data->power_limit_state;
}

void ChassisTriggerChangeUi::updateDbusData(const rm_msgs::DbusData::ConstPtr data)
{
  s_l_ = data->s_l;
  s_r_ = data->s_r;
  key_ctrl_ = data->key_ctrl;
  key_shift_ = data->key_shift;
  key_b_ = data->key_b;
}

void ChassisTriggerChangeUi::updateCapacityData(const rm_msgs::CapacityData data)
{
  displayInCapacity();
}

void ShooterTriggerChangeUi::display()
{
  updateConfig(shooter_mode_, 0, shoot_frequency_, false);
  TriggerChangeUi::display();
}

void ShooterTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
{
  graph_->setContent(getShooterState(main_mode));
  if (sub_mode == rm_common::HeatLimit::LOW)
    graph_->setColor(rm_referee::GraphColor::WHITE);
  else if (sub_mode == rm_common::HeatLimit::HIGH)
    graph_->setColor(rm_referee::GraphColor::YELLOW);
  else if (sub_mode == rm_common::HeatLimit::BURST)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
}

std::string ShooterTriggerChangeUi::getShooterState(uint8_t mode)
{
  if (mode == rm_msgs::ShootCmd::READY)
    return "ready";
  else if (mode == rm_msgs::ShootCmd::PUSH)
    return "push";
  else if (mode == rm_msgs::ShootCmd::STOP)
    return "stop";
  else
    return "error";
}

void ShooterTriggerChangeUi::updateShootCmdData(const rm_msgs::ShootCmd::ConstPtr data)
{
  shooter_mode_ = data->mode;
  display();
}

void ShooterTriggerChangeUi::updateManualCmdData(rm_msgs::ManualToReferee::ConstPtr data)
{
  shoot_frequency_ = data->shoot_frequency;
}

void GimbalTriggerChangeUi::display()
{
  updateConfig(gimbal_mode_, gimbal_eject_);
  graph_->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_->displayTwice(true);
  graph_->sendUi(ros::Time::now());
}

void GimbalTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
{
  graph_->setContent(getGimbalState(main_mode));
  if (main_flag)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
  else
    graph_->setColor(rm_referee::GraphColor::WHITE);
}

std::string GimbalTriggerChangeUi::getGimbalState(uint8_t mode)
{
  if (mode == rm_msgs::GimbalCmd::DIRECT)
    return "direct";
  else if (mode == rm_msgs::GimbalCmd::RATE)
    return "rate";
  else if (mode == rm_msgs::GimbalCmd::TRACK)
    return "track";
  else
    return "error";
}

void GimbalTriggerChangeUi::updateGimbalCmdData(const rm_msgs::GimbalCmd::ConstPtr data)
{
  gimbal_mode_ = data->mode;
  display();
}

void GimbalTriggerChangeUi::updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data)
{
  gimbal_eject_ = data->gimbal_eject;
}

void TargetTriggerChangeUi::display()
{
  if (base_.robot_id_ != rm_referee::RobotId::BLUE_HERO && base_.robot_id_ != rm_referee::RobotId::RED_HERO)
    updateConfig(det_target_, shoot_frequency_ == rm_common::HeatLimit::BURST, det_armor_target_,
                 det_color_ == rm_msgs::StatusChangeRequest::RED);
  else
    updateConfig(gimbal_eject_, shoot_frequency_, det_armor_target_, det_color_ == rm_msgs::StatusChangeRequest::RED);
  TriggerChangeUi::display();
}

void TargetTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
{
  graph_->setContent(getTargetState(main_mode, sub_mode));
  if (main_flag)
    graph_->setColor(rm_referee::GraphColor::ORANGE);
  else if (sub_flag)
    graph_->setColor(rm_referee::GraphColor::PINK);
  else
    graph_->setColor(rm_referee::GraphColor::CYAN);
}

std::string TargetTriggerChangeUi::getTargetState(uint8_t target, uint8_t armor_target)
{
  if (base_.robot_id_ != rm_referee::RobotId::BLUE_HERO && base_.robot_id_ != rm_referee::RobotId::RED_HERO)
  {
    if (target == rm_msgs::StatusChangeRequest::BUFF)
      return "buff";
    else if (target == rm_msgs::StatusChangeRequest::ARMOR && armor_target == rm_msgs::StatusChangeRequest::ARMOR_ALL)
      return "armor_all";
    else if (target == rm_msgs::StatusChangeRequest::ARMOR &&
             armor_target == rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE)
      return "armor_base";
    else
      return "error";
  }
  else
  {
    if (target == 1)
      return "eject";
    else if (armor_target == rm_msgs::StatusChangeRequest::ARMOR_ALL)
      return "all";
    else if (armor_target == rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE)
      return "base";
    else
      return "error";
  }
}

void TargetTriggerChangeUi::updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data)
{
  det_target_ = data->det_target;
  shoot_frequency_ = data->shoot_frequency;
  det_armor_target_ = data->det_armor_target;
  det_color_ = data->det_color;
  gimbal_eject_ = data->gimbal_eject;
}

void TargetTriggerChangeUi::updateShootCmdData(const rm_msgs::ShootCmd::ConstPtr data)
{
  display();
}

void BloodVolumeTriggerChangeUi::add()
{
  is_deleted_ = false;
  UiBase::add();
}

void BloodVolumeTriggerChangeUi::erasure()
{
  is_deleted_ = true;
  UiBase::erasure();
}

std::string BloodVolumeTriggerChangeUi::getRobotName(uint8_t id)
{
  if (id == rm_msgs::GameRobotStatus::RED_ENGINEER)
    return "RED_ENGINEER";
  else if (id == rm_msgs::GameRobotStatus::RED_SENTRY)
    return "RED_SENTRY";
  else if (id == rm_msgs::GameRobotStatus::RED_HERO)
    return "RED_HERO";
  else if (id == rm_msgs::GameRobotStatus::RED_STANDARD_3)
    return "RED_STANDARD3";
  else if (id == rm_msgs::GameRobotStatus::RED_STANDARD_4)
    return "RED_STANDARD4";
  else if (id == rm_msgs::GameRobotStatus::RED_STANDARD_5)
    return "RED_STANDARD5";
  else if (id == rm_msgs::GameRobotStatus::BLUE_ENGINEER)
    return "BLUE_ENGINEER";
  else if (id == rm_msgs::GameRobotStatus::BLUE_SENTRY)
    return "BLUE_SENTRY";
  else if (id == rm_msgs::GameRobotStatus::BLUE_HERO)
    return "BLUE_HERO";
  else if (id == rm_msgs::GameRobotStatus::BLUE_STANDARD_3)
    return "BLUE_STANDARD3";
  else if (id == rm_msgs::GameRobotStatus::BLUE_STANDARD_4)
    return "BLUE_STANDARD4";
  else if (id == rm_msgs::GameRobotStatus::BLUE_STANDARD_5)
    return "BLUE_STANDARD5";
  else
    return "error";
}

int BloodVolumeTriggerChangeUi::getRobotHp(uint8_t id)
{
  if (id == rm_msgs::GameRobotStatus::RED_ENGINEER)
    return robot_hp_.red_2_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::RED_SENTRY)
    return robot_hp_.red_7_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::RED_HERO)
    return robot_hp_.red_1_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::RED_STANDARD_3)
    return robot_hp_.red_3_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::RED_STANDARD_4)
    return robot_hp_.red_4_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::RED_STANDARD_5)
    return robot_hp_.red_5_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::BLUE_ENGINEER)
    return robot_hp_.blue_2_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::BLUE_SENTRY)
    return robot_hp_.blue_7_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::BLUE_HERO)
    return robot_hp_.blue_1_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::BLUE_STANDARD_3)
    return robot_hp_.blue_3_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::BLUE_STANDARD_4)
    return robot_hp_.blue_4_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::BLUE_STANDARD_5)
    return robot_hp_.blue_5_robot_hp;
  else
    return -1;
}

void BloodVolumeTriggerChangeUi::updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode, bool sub_flag)
{
  graph_->setColor(sub_flag ? rm_referee::GraphColor::PINK : rm_referee::GraphColor::CYAN);
  graph_->setTitle(getRobotName(main_mode) + ": ");
  graph_->setContent("+" + std::to_string(getRobotHp(main_mode)));

  if (!is_deleted_)
    display();
  else
    add();
}

void BloodVolumeTriggerChangeUi::updateTrackData(const rm_msgs::TrackData::ConstPtr data, const ros::Time& time)
{
  if (data->id > 100)
    updateConfig(data->id, true, 0, false);
  else if (data->id > 0)
    updateConfig(data->id, true, 1, true);
  else
    erasure();
}

int RobotInteractiveTrackTriggerChangeUi::getRobotHp(uint8_t id)
{
  if (id == rm_msgs::GameRobotStatus::RED_ENGINEER)
    return robot_hp_.red_2_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::RED_SENTRY)
    return robot_hp_.red_7_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::RED_HERO)
    return robot_hp_.red_1_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::RED_STANDARD_3)
    return robot_hp_.red_3_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::RED_STANDARD_4)
    return robot_hp_.red_4_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::RED_STANDARD_5)
    return robot_hp_.red_5_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::BLUE_ENGINEER)
    return robot_hp_.blue_2_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::BLUE_SENTRY)
    return robot_hp_.blue_7_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::BLUE_HERO)
    return robot_hp_.blue_1_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::BLUE_STANDARD_3)
    return robot_hp_.blue_3_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::BLUE_STANDARD_4)
    return robot_hp_.blue_4_robot_hp;
  else if (id == rm_msgs::GameRobotStatus::BLUE_STANDARD_5)
    return robot_hp_.blue_5_robot_hp;
  else
    return -1;
}

std::string RobotInteractiveTrackTriggerChangeUi::getRobotName(uint8_t id)
{
  if (id == rm_msgs::GameRobotStatus::RED_ENGINEER)
    return "RED_ENGINEER";
  else if (id == rm_msgs::GameRobotStatus::RED_SENTRY)
    return "RED_SENTRY";
  else if (id == rm_msgs::GameRobotStatus::RED_HERO)
    return "RED_HERO";
  else if (id == rm_msgs::GameRobotStatus::RED_STANDARD_3)
    return "RED_S3";
  else if (id == rm_msgs::GameRobotStatus::RED_STANDARD_4)
    return "RED_S4";
  else if (id == rm_msgs::GameRobotStatus::RED_STANDARD_5)
    return "RED_S5";
  else if (id == rm_msgs::GameRobotStatus::BLUE_ENGINEER)
    return "BLUE_ENGINEER";
  else if (id == rm_msgs::GameRobotStatus::BLUE_SENTRY)
    return "BLUE_SENTRY";
  else if (id == rm_msgs::GameRobotStatus::BLUE_HERO)
    return "BLUE_HERO";
  else if (id == rm_msgs::GameRobotStatus::BLUE_STANDARD_3)
    return "BLUE_S3";
  else if (id == rm_msgs::GameRobotStatus::BLUE_STANDARD_4)
    return "BLUE_S4";
  else if (id == rm_msgs::GameRobotStatus::BLUE_STANDARD_5)
    return "BLUE_S5";
  else
    return "error";
}

std::string RobotInteractiveTrackTriggerChangeUi::getInformation(int sender_id, int track_id)
{
  return (" " + std::to_string(getRobotHp(sender_id)) + " " + std::to_string(getRobotHp(track_id)) + " " +
          getRobotName(track_id));
}

void RobotInteractiveTrackTriggerChangeUi::add()
{
  robot_track_graph0->setOperation(rm_referee::GraphOperation::ADD);
  robot_track_graph0->display(true);
  robot_track_graph0->sendUi(ros::Time::now());
  robot_track_graph1->setOperation(rm_referee::GraphOperation::ADD);
  robot_track_graph1->display(true);
  robot_track_graph1->sendUi(ros::Time::now());
  robot_track_graph2->setOperation(rm_referee::GraphOperation::ADD);
  robot_track_graph2->display(true);
  robot_track_graph2->sendUi(ros::Time::now());
  robot_track_graph3->setOperation(rm_referee::GraphOperation::ADD);
  robot_track_graph3->display(true);
  robot_track_graph3->sendUi(ros::Time::now());
}

void RobotInteractiveTrackTriggerChangeUi::erasure()
{
  robot_track_graph0->setOperation(rm_referee::GraphOperation::DELETE);
  robot_track_graph0->display(true);
  robot_track_graph0->sendUi(ros::Time::now());
  robot_track_graph1->setOperation(rm_referee::GraphOperation::DELETE);
  robot_track_graph1->display(true);
  robot_track_graph1->sendUi(ros::Time::now());
  robot_track_graph2->setOperation(rm_referee::GraphOperation::DELETE);
  robot_track_graph2->display(true);
  robot_track_graph2->sendUi(ros::Time::now());
  robot_track_graph3->setOperation(rm_referee::GraphOperation::DELETE);
  robot_track_graph3->display(true);
  robot_track_graph3->sendUi(ros::Time::now());
}

void RobotInteractiveTrackTriggerChangeUi::updateConfig(uint8_t ui_order, bool is_urgency, uint8_t sub_mode,
                                                        bool sub_flag)
{
  if (is_urgency)
    graph_vector_.at(ui_order)->setColor(rm_referee::GraphColor::PINK);
  else
    graph_vector_.at(ui_order)->setColor(rm_referee::GraphColor::GREEN);

  graph_vector_.at(ui_order)->setTitle(getRobotName(date_container_.at(ui_order).first));
  graph_vector_.at(ui_order)->setContent(
      getInformation(date_container_.at(ui_order).first, date_container_.at(ui_order).second));
  graph_vector_.at(ui_order)->setStartX(100);
  graph_vector_.at(ui_order)->setStartY(800 - 100 * ui_order);

  graph_vector_.at(ui_order)->setOperation(rm_referee::GraphOperation::UPDATE);
  graph_vector_.at(ui_order)->display(true);
  graph_vector_.at(ui_order)->sendUi(ros::Time::now());
}

void RobotInteractiveTrackTriggerChangeUi::updateTrackData(const rm_msgs::TrackData::ConstPtr data,
                                                           const ros::Time& time)
{
  static int last_send_data = -1;
  if (data->id != last_send_data && data->id >= 0)
  {
    if (base_.robot_id_ < 100)
    {
      interactive_sender_->sendInteractiveData(rm_referee::DataCmdId::ROBOT_INTERACTIVE_CMD_MIN +
                                                   rm_msgs::GameStatus::INTERACTIVE_DATA_TRACK,
                                               rm_msgs::GameRobotStatus::RED_HERO, data->id);
      interactive_sender_->sendInteractiveData(rm_referee::DataCmdId::ROBOT_INTERACTIVE_CMD_MIN +
                                                   rm_msgs::GameStatus::INTERACTIVE_DATA_TRACK,
                                               rm_msgs::GameRobotStatus::RED_STANDARD_3, data->id);
      interactive_sender_->sendInteractiveData(rm_referee::DataCmdId::ROBOT_INTERACTIVE_CMD_MIN +
                                                   rm_msgs::GameStatus::INTERACTIVE_DATA_TRACK,
                                               rm_msgs::GameRobotStatus::RED_STANDARD_4, data->id);
      interactive_sender_->sendInteractiveData(rm_referee::DataCmdId::ROBOT_INTERACTIVE_CMD_MIN +
                                                   rm_msgs::GameStatus::INTERACTIVE_DATA_TRACK,
                                               rm_msgs::GameRobotStatus::RED_STANDARD_5, data->id);
      interactive_sender_->sendInteractiveData(rm_referee::DataCmdId::ROBOT_INTERACTIVE_CMD_MIN +
                                                   rm_msgs::GameStatus::INTERACTIVE_DATA_TRACK,
                                               rm_msgs::GameRobotStatus::RED_ENGINEER, data->id);
    }
    else if (base_.robot_id_ > 100)
    {
      interactive_sender_->sendInteractiveData(rm_referee::DataCmdId::ROBOT_INTERACTIVE_CMD_MIN +
                                                   rm_msgs::GameStatus::INTERACTIVE_DATA_TRACK,
                                               rm_msgs::GameRobotStatus::BLUE_HERO, data->id);
      interactive_sender_->sendInteractiveData(rm_referee::DataCmdId::ROBOT_INTERACTIVE_CMD_MIN +
                                                   rm_msgs::GameStatus::INTERACTIVE_DATA_TRACK,
                                               rm_msgs::GameRobotStatus::BLUE_STANDARD_3, data->id);
      interactive_sender_->sendInteractiveData(rm_referee::DataCmdId::ROBOT_INTERACTIVE_CMD_MIN +
                                                   rm_msgs::GameStatus::INTERACTIVE_DATA_TRACK,
                                               rm_msgs::GameRobotStatus::BLUE_STANDARD_4, data->id);
      interactive_sender_->sendInteractiveData(rm_referee::DataCmdId::ROBOT_INTERACTIVE_CMD_MIN +
                                                   rm_msgs::GameStatus::INTERACTIVE_DATA_TRACK,
                                               rm_msgs::GameRobotStatus::BLUE_STANDARD_5, data->id);
      interactive_sender_->sendInteractiveData(rm_referee::DataCmdId::ROBOT_INTERACTIVE_CMD_MIN +
                                                   rm_msgs::GameStatus::INTERACTIVE_DATA_TRACK,
                                               rm_msgs::GameRobotStatus::BLUE_ENGINEER, data->id);
    }

    last_send_data = data->id;
  }
}

void RobotInteractiveTrackTriggerChangeUi::updateInteractiveData(const rm_referee::InteractiveData& interactive_data,
                                                                 const ros::Time& time)
{
  if (interactive_data.header_data_.data_cmd_id_ !=
      rm_referee::DataCmdId::ROBOT_INTERACTIVE_CMD_MIN + rm_msgs::GameStatus::INTERACTIVE_DATA_TRACK)
    return;
  if (date_container_.empty())
  {
    date_container_.push_back(std::make_pair(interactive_data.header_data_.sender_id_, interactive_data.data_));
  }

  for (unsigned int i = 0; i < date_container_.size(); i++)
  {
    if (i == date_container_.size() - 1 && date_container_.at(i).first != interactive_data.header_data_.sender_id_)
    {
      if (date_container_.size() < 4)
      {
        date_container_.push_back(std::make_pair(interactive_data.header_data_.sender_id_, interactive_data.data_));
      }
    }
    else if (date_container_.at(i).first == interactive_data.header_data_.sender_id_)
    {
      date_container_.at(i).second = interactive_data.data_;
    }
  }
  for (unsigned int i = 0; i < date_container_.size(); i++)
  {
    ROS_INFO("%d",date_container_.at(i).second = interactive_data.data_);
    updateConfig(i, getRobotHp(date_container_.at(i).first) < 30);
  }
}

}  // namespace rm_referee
