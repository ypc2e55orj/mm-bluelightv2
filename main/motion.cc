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
void Motion::calcSideWallAdjust(RunParameter &param, RunTarget &target) {
  // 左右の壁センサが制御に使用できる状態なら、目標値との差分を使って制御
}

// 更新
void Motion::update() {
  Sensed sensed{};
  RunParameter param{};
  RunTarget target{};

  float voltage_left = 0, voltage_right = 0;
  int battery_voltage = 0;

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

  // 横壁制御が出来る場合は計算
  calcSideWallAdjust(param, target);

  // フィードバック

  // モーター電圧の制限
  voltage_left = std::max(-1.0f * VOLTAGE_MOTOR_LIMIT, std::min(VOLTAGE_MOTOR_LIMIT, voltage_left));
  voltage_right = std::max(-1.0f * VOLTAGE_MOTOR_LIMIT, std::min(VOLTAGE_MOTOR_LIMIT, voltage_right));

  // モーター電圧を設定
  dri_->motor_left->speed(static_cast<int>(voltage_left * 1000.0f), battery_voltage);
  dri_->motor_right->speed(static_cast<int>(voltage_right * 1000.0f), battery_voltage);
}