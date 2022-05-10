//
// Created by luohx on 7/20/20.
//

#ifndef RM_MANUAL_COMMON_DATA_H_
#define RM_MANUAL_COMMON_DATA_H_

#include <ros/ros.h>
#include <serial/serial.h>
#include <rm_common/decision/target_cost_function.h>
#include <rm_common/referee/referee.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <rm_msgs/ActuatorState.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/StatusChange.h>
#include <std_msgs/Float64.h>
#include <rm_msgs/ShootCmd.h>
#include <rm_msgs/GimbalCmd.h>

namespace rm_referee
{
class Data
{
public:
  explicit Data(ros::NodeHandle& nh) : tf_listener_(tf_buffer_)
  {
    // sub
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &Data::dbusDataCallback, this);
    chassis_cmd_sub_ = nh.subscribe<rm_msgs::ChassisCmd>("/controllers/chassis_controller/command", 10,
                                                         &Data::chassiscmdDataCallback, this);
    vel_cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &Data::velcmdDataCallback, this);
    cover_cmd_sub_ =
        nh.subscribe<std_msgs::Float64>("/controllers/cover_controller/command", 10, &Data::covercmdDataCallback, this);
    shoot_cmd_sub_ = nh.subscribe<rm_msgs::ShootCmd>("/controllers/shooter_controller/command", 10,
                                                     &Data::shootcmdDataCallback, this);
    gimbal_cmd_sub_ = nh.subscribe<rm_msgs::GimbalCmd>("/controllers/gimbal_controller/command", 10,
                                                       &Data::gimbalcmdDataCallback, this);
    // client
    switch_detection_srv_ = nh.serviceClient<rm_msgs::StatusChange>("/detection_nodelet/status_switch");

    joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &Data::jointStateCallback, this);
    actuator_state_sub_ =
        nh.subscribe<rm_msgs::ActuatorState>("/actuator_states", 10, &Data::actuatorStateCallback, this);
    track_sub_ =
        nh.subscribe<rm_msgs::TrackDataArray>("/controllers/gimbal_controller/track", 10, &Data::trackCallback, this);
    gimbal_des_error_sub_ = nh.subscribe<rm_msgs::GimbalDesError>("/controllers/gimbal_controller/error_des", 10,
                                                                  &Data::gimbalDesErrorCallback, this);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &Data::odomCallback, this);

    // pub
    ros::NodeHandle root_nh;
    referee_.referee_pub_ = root_nh.advertise<rm_msgs::Referee>("/referee", 1);
    referee_.super_capacitor_pub_ = root_nh.advertise<rm_msgs::SuperCapacitor>("/super_capacitor", 1);
    initSerial();
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
  {
    joint_state_ = *joint_state;
  }

  void actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data)
  {
    actuator_state_ = *data;
  }

  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
  {
    dbus_data_ = *data;
  }

  void trackCallback(const rm_msgs::TrackDataArray::ConstPtr& data)
  {
    track_data_array_ = *data;
  }

  void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr& data)
  {
    gimbal_des_error_ = *data;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& data)
  {
    odom_ = *data;
  }

  void covercmdDataCallback(const std_msgs::Float64::ConstPtr& data)
  {
    cover_cmd_data_ = *data;
  }

  void chassiscmdDataCallback(const rm_msgs::ChassisCmd::ConstPtr& data)
  {
    chassis_cmd_data_ = *data;
  }

  void velcmdDataCallback(const geometry_msgs::Twist::ConstPtr& data)
  {
    vel_cmd_data_ = *data;
  }

  void shootcmdDataCallback(const rm_msgs::ShootCmd::ConstPtr& data)
  {
    shoot_cmd_data_ = *data;
  }

  void gimbalcmdDataCallback(const rm_msgs::GimbalCmd::ConstPtr& data)
  {
    gimbal_cmd_data_ = *data;
  }

  void initSerial()
  {
    serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
    serial_.setPort("/dev/usbReferee");
    serial_.setBaudrate(115200);
    serial_.setTimeout(timeout);
    if (serial_.isOpen())
      return;
    try
    {
      serial_.open();
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR("Cannot open referee port");
    }
  }

  ros::Subscriber dbus_sub_;
  ros::Subscriber chassis_cmd_sub_;
  ros::Subscriber vel_cmd_sub_;
  ros::Subscriber cover_cmd_sub_;
  ros::Subscriber shoot_cmd_sub_;
  ros::Subscriber gimbal_cmd_sub_;

  ros::Subscriber joint_state_sub_;
  ros::Subscriber actuator_state_sub_;
  ros::Subscriber track_sub_;
  ros::Subscriber gimbal_des_error_sub_;
  ros::Subscriber odom_sub_;

  ros::ServiceClient switch_detection_srv_;

  rm_msgs::DbusData dbus_data_;
  rm_msgs::ChassisCmd chassis_cmd_data_;
  geometry_msgs::Twist vel_cmd_data_;
  std_msgs::Float64 cover_cmd_data_;
  rm_msgs::StatusChange switch_detection_data_;
  rm_msgs::ShootCmd shoot_cmd_data_;
  rm_msgs::GimbalCmd gimbal_cmd_data_;

  sensor_msgs::JointState joint_state_;
  rm_msgs::ActuatorState actuator_state_;
  rm_msgs::TrackDataArray track_data_array_;
  rm_msgs::GimbalDesError gimbal_des_error_;
  nav_msgs::Odometry odom_;

  rm_common::Referee referee_;
  serial::Serial serial_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace rm_referee
#endif  // RM_MANUAL_COMMON_DATA_H_
