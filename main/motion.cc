#include "motion.h"

// C++
#include <cmath>

// コンストラクタ
Motion::Motion(Driver *dri, Sensor *sensor) : driver_(dri), sensor_(sensor) {}
// デストラクタ
Motion::~Motion() = default;

// 目標値を計算する
void Motion::calcTarget(MotionParameter &param, MotionTarget &target) {
  switch (param.pattern) {
    case MotionPattern::Turn: {
      // 角加速度から目標角速度を生成
      target.angular_velocity += param.max_angular_velocity / 1000.0f;
      // 制限
      if (std::abs(target.angular_velocity) > param.max_angular_velocity) {
        target.angular_velocity = (std::signbit(target.angular_velocity) ? -1.0f : 1.0f) * param.max_angular_velocity;
      }
    }
      [[fallthrough]];

    case MotionPattern::Straight:
      // 加速度から目標速度を生成
      target.velocity += param.acceleration / 1000.0f;
      // 制限
      if (target.velocity > param.max_velocity) {
        target.velocity = param.max_velocity;
      }
      break;

    case MotionPattern::Stop:
      break;
  }
}

// 横壁からの目標値を計算する
bool Motion::calcSideWallAdjust(const MotionParameter &param, const Sensed &sensed, MotionTarget &target) {
  if (param.pattern != MotionPattern::Straight) {
    // 壁制御できない
    wall_adj_side_pid_.reset();
    return false;
  }
  // 左右の壁センサが制御に使用できる状態なら、目標値との差分を使って制御
  int error;
  if (sensed.wall_left45.exist && sensed.wall_right45.exist) {
    // 両方の壁が使える場合はそのまま
    error = sensed.wall_left45.error - sensed.wall_right45.error;
  } else if (sensed.wall_left45.exist || sensed.wall_right90.exist) {
    // 片方の壁だけが使える場合は2倍
    error = (sensed.wall_left45.error - sensed.wall_right45.error) * 2;
  } else {
    // 壁制御できない
    wall_adj_side_pid_.reset();
    return false;
  }
  target.angular_velocity = wall_adj_side_pid_.update(0.0f, static_cast<float>(error), 1.0f);
  return true;
}

std::pair<float, float> Motion::calcFeedback(const MotionParameter &param, const Sensed &sensed, MotionTarget &target) {
  switch (param.pattern) {
    case MotionPattern::Straight:
    case MotionPattern::Turn: {
      auto velo = velo_pid_.update(target.velocity, sensed.velocity, 1.0f);
      auto ang_velo = ang_velo_pid_.update(target.angular_velocity, sensed.angular_velocity, 1.0f);

      return {velo + ang_velo, velo - ang_velo};
    }

    default:
      velo_pid_.reset();
      ang_velo_pid_.reset();

      return {0.0f, 0.0f};
  }
}

// 更新
void Motion::update() {
  MotionParameter param{};

  // 走行パラメータがあれば取得
  if (param_queue_.receive(&param, 0)) {
  }
  // 走行パターンに応じた目標速度生成
  calcTarget(param, target_);
  // 横壁制御が出来る場合は目標角度生成
  if (param.enable_side_wall_adjust) {
    calcSideWallAdjust(param, sensor_->getSensed(), target_);
  }

  // フィードバック
  auto [left, right] = calcFeedback(param, sensor_->getSensed(), target_);

  // モーター電圧の制限
  left = std::max(-1.0f * VOLTAGE_MOTOR_LIMIT, std::min(VOLTAGE_MOTOR_LIMIT, left));
  right = std::max(-1.0f * VOLTAGE_MOTOR_LIMIT, std::min(VOLTAGE_MOTOR_LIMIT, right));

  // モーター電圧を設定
  driver_->motor_left->speed(static_cast<int>(left * 1000.0f), sensor_->getSensed().battery_voltage);
  driver_->motor_right->speed(static_cast<int>(right * 1000.0f), sensor_->getSensed().battery_voltage);
}