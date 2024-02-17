#include "run.h"

// コンストラクタ
Run::Run(Driver *driver, Sensor *sensor, Motion *motion) : driver_(driver), sensor_(sensor), motion_(motion) {}
// デストラクタ
Run::~Run() = default;

// 走行
void Run::straight(float length, float acceleration, float max_velocity, float end_velocity) {
  constexpr auto LENGTH_OFFSET = 10.0f;

  MotionParameter param{};
  param.pattern = MotionPattern::Straight;
  param.acceleration = acceleration;
  param.max_velocity = max_velocity;
  param.end_velocity = end_velocity;
  param.max_length = length;
  param.enable_side_wall_adjust = false;

  driver_->motor_right->enable();
  driver_->motor_left->enable();

  motion_->getParameterQueue().overwrite(&param);
  // 完了まで待つ

  driver_->motor_left->disable();
  driver_->motor_right->disable();
}
void Run::turn(float degree, float angular_acceleration, float max_angular_velocity, RunDirection dir) {
  auto &sensed = sensor_->getSensed();
  auto &target = motion_->getTarget();
}
