//
// Created by qiayuan on 5/22/21.
//

#pragma once

#include "rm_referee/common/referee_base.h"

namespace rm_referee
{
class ChassisGimbalReferee : public RefereeBase
{
public:
  explicit ChassisGimbalReferee(ros::NodeHandle& nh);

protected:
  void drawUi(const ros::Time& time) override;

  TimeChangeUi* time_change_ui_{};
  FlashUi* flash_ui_{};
  TriggerChangeUi* trigger_change_ui_{};
  FixedUi* fixed_ui_{};
  GimbalChassisUi* gimbal_chassis_ui_{};
};
}  // namespace rm_referee
