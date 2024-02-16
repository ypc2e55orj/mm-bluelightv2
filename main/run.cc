#include "run.h"

// コンストラクタ
Run::Run(Driver *driver, Sensor *sensor, Motion *motion) : driver_(driver), sensor_(sensor), motion_(motion) {}
// デストラクタ
Run::~Run() = default;

// 走行
void Run::straight(float length, float acceleration, float max_velocity, float end_velocity) {
  constexpr auto LENGTH_OFFSET = 10.0f;

  auto &sensed = sensor_->getSensed();
  auto &target = motion_->getTarget();
  auto &param = motion_->getParameter();

  sensor_->reset();
  motion_->reset();

  param.enable_side_wall_adjust = false;
  param.acceleration = acceleration;
  param.max_velocity = max_velocity;
  param.pattern = MotionPattern::Straight;

  driver_->motor_right->enable();
  driver_->motor_left->enable();

  // 台形加速→等速待ち
  // 2as = v^2 - v0^2
  while (length - LENGTH_OFFSET - sensed.length >
         1000.0f * (target.velocity * target.velocity - end_velocity * end_velocity) / 2.0f * param.acceleration) {
    printf("a %f, %f\n", sensed.length, target.velocity);
  }
  // 減速
  param.acceleration = -1.0f * acceleration;
  auto offset = end_velocity == 0.0f ? 1.0f : 0.0f;
  while (sensed.length < length - offset) {
    printf("d %f, %f\n", sensed.length, target.velocity);
    if (target.velocity < VELOCITY_MIN) {
      param.acceleration = 0.0f;
      param.max_velocity = VELOCITY_MIN;
    }
  }
  // 終了速度が停止の場合は停止するまで待つ
  if (end_velocity == 0.0f) {
    param.max_velocity = 0;
    // 停止まで待つ
    while (sensed.velocity >= 0.0f) {
      printf("s %f, %f\n", sensed.length, target.velocity);
    }
  }
  // 完了
  param.acceleration = 0;
  param.pattern = MotionPattern::Stop;

  driver_->motor_left->disable();
  driver_->motor_right->disable();
}
void Run::turn(float degree, float angular_acceleration, float max_angular_velocity, RunDirection dir) {
  auto &sensed = sensor_->getSensed();
  auto &target = motion_->getTarget();

  MotionParameter param{};
  param.pattern = MotionPattern::Turn;
  param.enable_side_wall_adjust = false;
  param.max_angular_velocity = max_angular_velocity;
  param.angular_acceleration = angular_acceleration;
}
