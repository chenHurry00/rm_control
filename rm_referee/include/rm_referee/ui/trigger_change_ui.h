//
// Created by llljjjqqq on 22-11-4.
//
#pragma once

#include "rm_referee/ui/ui_base.h"
#include <rm_common/decision/power_limit.h>

namespace rm_referee
{
class TriggerChangeUi : public UiBase
{
public:
  explicit TriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, const std::string& graph_name) : UiBase(base)
  {
    if (graph_name == "chassis")
      graph_ = new Graph(rpc_value["config"], base_, 1);
    else
      graph_ = new Graph(rpc_value["config"], base_, id_++);
  }
  virtual void setContent(const std::string& content);
  virtual void display();
  virtual void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false){};
};
class ChassisTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit ChassisTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TriggerChangeUi(rpc_value, base, "chassis")
  {
    if (base.robot_id_ == rm_referee::RobotId::RED_ENGINEER || base.robot_id_ == rm_referee::RobotId::BLUE_ENGINEER)
      graph_->setContent("raw");
    else
      graph_->setContent("follow");
  }
  void updateChassisCmdData(const rm_msgs::ChassisCmd::ConstPtr data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data) override;
  void updateDbusData(const rm_msgs::DbusData::ConstPtr data);
  void updateCapacityData(const rm_msgs::CapacityData data);

private:
  void display() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  void displayInCapacity();
  std::string getChassisState(uint8_t mode);
  uint8_t chassis_mode_, power_limit_state_, s_l_, s_r_, key_ctrl_, key_shift_, key_b_;
};

class ShooterTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit ShooterTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TriggerChangeUi(rpc_value, base, "shooter")
  {
    graph_->setContent("0");
  }
  void updateShootCmdData(const rm_msgs::ShootCmd::ConstPtr data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data) override;

private:
  void display() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  std::string getShooterState(uint8_t mode);
  uint8_t shooter_mode_, shoot_frequency_;
};

class GimbalTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit GimbalTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TriggerChangeUi(rpc_value, base, "gimbal")
  {
    graph_->setContent("0");
  }
  void updateGimbalCmdData(const rm_msgs::GimbalCmd ::ConstPtr data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data) override;

private:
  void display() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  std::string getGimbalState(uint8_t mode);
  uint8_t gimbal_mode_, gimbal_eject_;
};

class TargetTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit TargetTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TriggerChangeUi(rpc_value, base, "target")
  {
    graph_->setContent("armor");
    if (base_.robot_color_ == "red")
      graph_->setColor(rm_referee::GraphColor::CYAN);
    else
      graph_->setColor(rm_referee::GraphColor::PINK);
  }
  void updateShootCmdData(const rm_msgs::ShootCmd::ConstPtr data);
  void updateManualCmdData(const rm_msgs::ManualToReferee::ConstPtr data) override;

private:
  void display() override;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
  std::string getTargetState(uint8_t target, uint8_t armor_target);
  uint8_t det_target_, shoot_frequency_, det_armor_target_, det_color_, gimbal_eject_;
};

class BloodVolumeTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit BloodVolumeTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TriggerChangeUi(rpc_value, base, "blood_volume"){};
  void add() override;
  void erasure() override;
  std::string getRobotName(uint8_t id);
  int getRobotHp(uint8_t id);
  void updateRobotHpDate(const rm_msgs::GameRobotHp data)
  {
    robot_hp_ = data;
  }
  void updateTrackData(const rm_msgs::TrackData::ConstPtr data, const ros::Time& time);

private:
  bool is_deleted_;
  int next_pos_x_, next_pos_y_;
  rm_msgs::GameRobotHp robot_hp_;
  void updateConfig(uint8_t main_mode, bool main_flag, uint8_t sub_mode = 0, bool sub_flag = false) override;
};

class RobotInteractiveTrackTriggerChangeUi : public TriggerChangeUi
{
public:
  explicit RobotInteractiveTrackTriggerChangeUi(XmlRpc::XmlRpcValue& rpc_value, Base& base)
    : TriggerChangeUi(rpc_value, base, "robot_interactive_track")
  {
    robot_track_graph0 = graph_;
    robot_track_graph1 = new Graph(rpc_value["config"], base_, UiBase::id_++);
    robot_track_graph2 = new Graph(rpc_value["config"], base_, UiBase::id_++);
    robot_track_graph3 = new Graph(rpc_value["config"], base_, UiBase::id_++);

    graph_vector_.push_back(robot_track_graph0);
    graph_vector_.push_back(robot_track_graph1);
    graph_vector_.push_back(robot_track_graph2);
    graph_vector_.push_back(robot_track_graph3);

    interactive_sender_ = new Graph(base_);
  }
  void add() override;
  void erasure() override;
  std::string getRobotName(uint8_t id);
  std::string getInformation(int sender_id, int track_id);
  int getRobotHp(uint8_t id);
  void updateInteractiveData(const rm_referee::InteractiveData& interactive_data, const ros::Time& time);
  void updateRobotHpDate(const rm_msgs::GameRobotHp data)
  {
    robot_hp_ = data;
  }
  void updateTrackData(const rm_msgs::TrackData::ConstPtr data, const ros::Time& time);

private:
  std::vector<std::pair<int, int>> date_container_;
  std::vector<Graph*> graph_vector_;
  Graph *robot_track_graph0, *robot_track_graph1, *robot_track_graph2, *robot_track_graph3;
  Graph* interactive_sender_;
  rm_msgs::GameRobotHp robot_hp_;
  void updateConfig(uint8_t ui_order, bool is_urgency, uint8_t sub_mode = 0, bool sub_flag = false) override;
};

}  // namespace rm_referee
