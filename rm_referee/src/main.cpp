//
// Created by luohx on 7/27/20.
//

#include "rm_referee/common/referee_base.h"
#include "rm_referee/chassis_gimbal_shooter_cover_referee.h"

int main(int argc, char** argv)
{
  std::string robot;
  rm_referee::RefereeBase* referee_control;
  ros::init(argc, argv, "rm_referee");
  ros::NodeHandle nh("~");
  robot = getParam(nh, "robot_type", (std::string) "error");
  if (robot == "standard")
    referee_control = new rm_referee::ChassisGimbalShooterCoverReferee(nh);
  else if (robot == "hero")
    referee_control = new rm_referee::ChassisGimbalShooterReferee(nh);
  else
  {
    ROS_ERROR("no robot type ");
    return 0;
  }
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    referee_control->run();
    loop_rate.sleep();
  }
  return 0;
}
