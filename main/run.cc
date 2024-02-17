#include "run.h"

// コンストラクタ
Run::Run(Driver *driver, Motion *motion) : driver_(driver), motion_(motion) {}
// デストラクタ
Run::~Run() = default;

// 走行
void Run::straight(float length, float acceleration, float max_velocity, float end_velocity) {
  MotionParameter param{};
  param.pattern = MotionPattern::Straight;
  param.acceleration = acceleration;
  param.min_velocity = VELOCITY_MIN;
  param.max_velocity = max_velocity;
  param.end_velocity = end_velocity;
  param.max_length = length;
  param.enable_side_wall_adjust = false;

  driver_->motor_right->enable();
  driver_->motor_left->enable();

  motion_->getParameterQueue().overwrite(&param);
  // 完了まで待つ
  motion_->wait();

  driver_->motor_left->disable();
  driver_->motor_right->disable();
}
void Run::turn(float degree, float angular_acceleration, float max_angular_velocity, MotionTurnDirection dir) {
  MotionParameter param{};
  param.pattern = MotionPattern::Turn;
  param.direction = dir;
  param.angular_acceleration = angular_acceleration;
  param.min_angular_velocity = ANGULAR_VELOCITY_MIN;
  param.max_angular_velocity = max_angular_velocity;
  param.max_angle = degree;

  driver_->motor_right->enable();
  driver_->motor_left->enable();

  motion_->getParameterQueue().overwrite(&param);
  // 完了まで待つ
  motion_->wait();

  driver_->motor_left->disable();
  driver_->motor_right->disable();
}
