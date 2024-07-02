#include "run.h"

// コンストラクタ
Run::Run(Motion *motion) : motion_(motion) {}
// デストラクタ
Run::~Run() = default;

// 走行
void Run::straight(MotionDirection direction, float length, float acceleration, float max_velocity,
                   float end_velocity) {
  MotionParameter param{};

  param.pattern = MotionPattern::Straight;
  param.direction = direction;
  param.acceleration = acceleration;
  param.start_velocity = VELOCITY_MIN;
  param.max_velocity = max_velocity;
  param.end_velocity = end_velocity;
  param.length = length;
  param.enable_side_wall_adjust = false;

  motion_->getParameterQueue().overwrite(&param);
  // 完了まで待つ
  motion_->wait();
}
void Run::turn(float degree, float angular_acceleration, float max_angular_velocity, MotionDirection dir) {
  MotionParameter param{};

  param.pattern = MotionPattern::Turn;
  param.direction = dir;
  param.angular_acceleration = angular_acceleration;
  param.start_angular_velocity = ANGULAR_VELOCITY_MIN;
  param.max_angular_velocity = max_angular_velocity;
  param.angle = degree;

  motion_->getParameterQueue().overwrite(&param);
  // 完了まで待つ
  motion_->wait();
}
void Run::stop() {
  MotionParameter param{};

  param.pattern = MotionPattern::Stop;

  motion_->getParameterQueue().overwrite(&param);
  // 完了まで待つ
  motion_->wait();
}
