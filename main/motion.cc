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
  if (sensed.wall_right45.exist && sensed.wall_left45.exist) {
    // 両方の壁が使える場合はそのまま
    error = sensed.wall_right45.error - sensed.wall_left45.error;
  } else if (sensed.wall_right90.exist || sensed.wall_left45.exist) {
    // 片方の壁だけが使える場合は2倍
    error = (sensed.wall_right45.error - sensed.wall_left45.error) * 2;
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
      // https://rt-net.jp/mobility/archives/16525
      // 速度・角速度でPIDフィードバック
      auto velo = target.velocity + velo_pid_.update(target.velocity, sensed.velocity, 1.0f);
      auto ang_velo =
          target.angular_velocity + ang_velo_pid_.update(target.angular_velocity, sensed.angular_velocity, 1.0f);
      // 目標車輪角速度を算出
      auto omega_r = velo / (TIRE_DIAMETER / 2.0f) + ang_velo * (TREAD_WIDTH / TIRE_DIAMETER);
      auto omega_l = velo / (TIRE_DIAMETER / 2.0f) - ang_velo * (TREAD_WIDTH / TIRE_DIAMETER);
      // rpmに換算
      auto rpm_r = 30 * omega_r / std::numbers::pi_v<float>;
      auto rpm_l = 30 * omega_l / std::numbers::pi_v<float>;
      // モーター電圧を計算
      auto e_r = MOTOR_KE * rpm_r;
      auto e_l = MOTOR_KE * rpm_l;

      return {e_r, e_l};
    }

    default:
      velo_pid_.reset();
      ang_velo_pid_.reset();

      return {0.0f, 0.0f};
  }
}

// 更新
void Motion::update() {
  // 走行パラメータがあれば取得
  if (param_queue_.receive(&param_, 0)) {
    if (param_.reset_pid) {
      velo_pid_.reset();
      ang_velo_pid_.reset();
    }
    if (param_.reset_sensor) {
      sensor_->reset();
    }
  }
  // 走行パターンに応じた目標速度生成
  calcTarget(param_, target_);
  // 横壁制御が出来る場合は目標角度生成
  if (param_.enable_side_wall_adjust) {
    calcSideWallAdjust(param_, sensor_->getSensed(), target_);
  }

  // フィードバック
  auto [right, left] = calcFeedback(param_, sensor_->getSensed(), target_);

  // モーター電圧の制限
  right = std::max(-1.0f * VOLTAGE_MOTOR_LIMIT, std::min(VOLTAGE_MOTOR_LIMIT, right));
  left = std::max(-1.0f * VOLTAGE_MOTOR_LIMIT, std::min(VOLTAGE_MOTOR_LIMIT, left));

  // モーター電圧を設定
  driver_->motor_right->speed(static_cast<int>(right * 1000.0f), sensor_->getSensed().battery_voltage);
  driver_->motor_left->speed(static_cast<int>(left * 1000.0f), sensor_->getSensed().battery_voltage);
}