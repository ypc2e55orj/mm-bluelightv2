#include "motion.h"

// C++
#include <cmath>
#include <numbers>

// コンストラクタ
Motion::Motion(Driver *dri, rtos::Queue<Sensed> &sensed_queue) : dri_(dri), sensed_queue_(sensed_queue) {}
// デストラクタ
Motion::~Motion() = default;

// 目標値を計算する
void Motion::calcTarget(RunParameter &param, RunTarget &target) {
  switch (param.pattern) {
    case RunPattern::PivotTurnLeft:
    case RunPattern::PivotTurnRight:
    case RunPattern::TurnLeft:
    case RunPattern::TurnRight: {
      // 角加速度から目標角速度を生成
      target.angular_velocity = target.angular_velocity + param.max_angular_velocity / 1000.0f;
      // 目標角速度から目標角度を生成
      target.angle = target.angle + (target.angular_velocity * 180.0f / std::numbers::pi_v<float>) / 1000;
      // 制限
      float sign = param.pattern == RunPattern::PivotTurnLeft || param.pattern == RunPattern::TurnLeft ? -1.0f : 1.0f;
      if (std::abs(target.angular_velocity) > param.max_angular_velocity) {
        target.angular_velocity = sign * param.max_angular_velocity;
      }
      if (std::abs(target.angle) > param.max_angle) {
        target.angle = sign * param.max_angle;
      }
    }
      [[fallthrough]];

    case RunPattern::Straight:
      // 加速度から目標速度を生成
      target.velocity = target.velocity + param.acceleration / 1000.0f;
      // 制限
      if (target.velocity > param.max_velocity) {
        target.velocity = param.max_velocity;
      }
      break;

    case RunPattern::FrontAdjust:
    case RunPattern::Stop:
      break;
  }
}

// 横壁からの目標値を計算する
bool Motion::calcSideWallAdjust(RunParameter &param, RunTarget &target, Sensed &sensed) {
  if (param.pattern != RunPattern::Straight) {
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

std::pair<float, float> Motion::calcFeedback(RunParameter &param, RunTarget &target, Sensed &sensed) {
  switch (param.pattern) {
    case RunPattern::Straight:
    case RunPattern::PivotTurnLeft:
    case RunPattern::PivotTurnRight:
    case RunPattern::TurnLeft:
    case RunPattern::TurnRight: {
      auto velo = velo_pid_.update(target.velocity, sensed.velocity, 0.0f);
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
  Sensed sensed{};
  RunParameter param{};
  RunTarget target{};

  // センサー値を取得
  if (!sensed_queue_.peek(&sensed, pdMS_TO_TICKS(1))) {
    return;
  }
  // 走行パラメータを取得
  if (!param_queue_.peek(&param, pdMS_TO_TICKS(1))) {
    return;
  }
  // 目標値を取得
  if (!target_queue_.peek(&target, pdMS_TO_TICKS(1))) {
    return;
  }

  // 走行パターンに応じた目標速度生成
  calcTarget(param, target);

  // 横壁制御が出来る場合は目標角度生成
  if (!calcSideWallAdjust(param, target, sensed)) {
  }

  // フィードバック
  auto [left, right] = calcFeedback(param, target, sensed);

  // モーター電圧の制限
  left = std::max(-1.0f * VOLTAGE_MOTOR_LIMIT, std::min(VOLTAGE_MOTOR_LIMIT, left));
  right = std::max(-1.0f * VOLTAGE_MOTOR_LIMIT, std::min(VOLTAGE_MOTOR_LIMIT, right));

  // モーター電圧を設定
  dri_->motor_left->speed(static_cast<int>(left * 1000.0f), sensed.battery_voltage);
  dri_->motor_right->speed(static_cast<int>(right * 1000.0f), sensed.battery_voltage);

  // 目標値を上書き
  if (!target_queue_.overwrite(&target)) {
    // 異常
  }
}