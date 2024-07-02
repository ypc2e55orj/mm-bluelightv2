#include "motion.h"

// C++
#include <cmath>

// コンストラクタ
Motion::Motion(Driver *dri, Sensor *sensor) : driver_(dri), sensor_(sensor) {}
// デストラクタ
Motion::~Motion() = default;

// 走行パターンに応じて現在のパラメータを設定
bool Motion::setPatternParam(const Sensed &sensed, const MotionParameter &param, MotionTarget &target) {
  switch (param.pattern) {
    case MotionPattern::Straight:
      return straight(sensed, param, target);

    case MotionPattern::Turn:
      return turn(sensed, param, target);

    case MotionPattern::Stop:
      return false;

    case MotionPattern::Free:
    default:
      return true;
  }
}
// 直線
bool Motion::straight(const Sensed &sensed, const MotionParameter &param, MotionTarget &target) {
  float sign = 1.0f;  // param.direction == MotionDirection::Forward ? 1.0f : -1.0f;
  float accel_length = (param.max_velocity * param.max_velocity - param.start_velocity * param.start_velocity) /
                       (2.0f * param.acceleration);
  float deccel_length =
      (param.max_velocity * param.max_velocity - param.end_velocity * param.end_velocity) / (2.0f * param.acceleration);

  // 加速
  if (accel_length > sign * sensed.length) {
    target.acceleration = sign * param.acceleration;
    target.max_velocity = sign * param.max_velocity;
    return false;
  }
  // 等速
  else if (param.length - deccel_length > sign * sensed.length) {
    target.acceleration = sign * param.acceleration;
    target.max_velocity = sign * param.max_velocity;
    return false;
  }
  // 減速
  else if (param.length > sign * sensed.length) {
    target.acceleration = -1.0f * sign * param.acceleration;
    if (sign * target.velocity < VELOCITY_MIN) {
      target.acceleration = 0.0f;
      target.velocity = sign * VELOCITY_MIN;
    }
    return false;
  }

  target.acceleration = 0.0f;
  target.velocity = sign * param.end_velocity;

  return true;
}
// 旋回
bool Motion::turn(const Sensed &sensed, const MotionParameter &param, MotionTarget &target) {
  float sign = param.direction == MotionDirection::Right ? -1.0f : 1.0f;

  return true;
}

// 目標値を計算する
void Motion::calcTargetVelocity(MotionPattern pattern, MotionTarget &target) {
  switch (pattern) {
    case MotionPattern::Turn: {
      // 角加速度から目標角速度を生成
      target.angular_velocity += target.angular_acceleration / 1000.0f;
      // 制限
      if (std::abs(target.angular_velocity) > std::abs(target.max_angular_velocity)) {
        target.angular_velocity = target.max_angular_velocity;
      }
    }
      [[fallthrough]];

    case MotionPattern::Straight:
      // 加速度から目標速度を生成
      target.velocity += target.acceleration / 1000.0f;
      // 制限
      if (std::abs(target.velocity) > std::abs(target.max_velocity)) {
        target.velocity = target.max_velocity;
      }
      break;

    case MotionPattern::Stop:
    case MotionPattern::Free:
      break;
  }
}

// 横壁からの目標値を計算する
bool Motion::adjustSideWall(const Sensed &sensed, const MotionParameter &param, MotionTarget &target) {
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

// 目標速度と現在速度のフィードバック値を計算する
std::pair<float, float> Motion::calcMotorVoltage(const Sensed &sensed, const MotionParameter &param,
                                                 MotionTarget &target) {
  switch (param.pattern) {
    case MotionPattern::Stop:
    case MotionPattern::Straight:
    case MotionPattern::Turn: {
      // https://rt-net.jp/mobility/archives/16525
      // 速度・角速度でPIDフィードバック
      auto velo = target.velocity + velo_pid_.update(target.velocity, sensed.velocity, 1.0f);
      auto ang_velo =
          target.angular_velocity + ang_velo_pid_.update(target.angular_velocity, sensed.angular_velocity, 1.0f);
      // 目標車輪角速度を算出
      auto omega_r = (velo * 1000.0f) / TIRE_RADIUS + (TREAD_WIDTH / TIRE_DIAMETER) * ang_velo;
      auto omega_l = (velo * 1000.0f) / TIRE_RADIUS - (TREAD_WIDTH / TIRE_DIAMETER) * ang_velo;
      // モーター電圧を計算
      auto e_r = MOTOR_KE * omega_r;
      auto e_l = MOTOR_KE * omega_l;

      return {e_r, e_l};
    }

    case MotionPattern::Free:
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
    // 制御則が切り替わるのでリセット
    velo_pid_.reset();
    ang_velo_pid_.reset();
    sensor_->reset();
  }

  // 走行パターンに応じて現在のパラメータを設定
  if (handle_notify_ != nullptr && setPatternParam(sensor_->getSensed(), param_, target_)) {
    // 完了通知
    xTaskNotifyGive(handle_notify_);
  }

  // 走行パターンに応じた目標速度生成
  calcTargetVelocity(param_.pattern, target_);
  // 横壁制御が出来る場合は目標角度生成
  if (param_.enable_side_wall_adjust) {
    adjustSideWall(sensor_->getSensed(), param_, target_);
  }

  // フィードバック
  auto [right, left] = calcMotorVoltage(sensor_->getSensed(), param_, target_);

  // モーター電圧の制限
  right = std::max(-1.0f * VOLTAGE_MOTOR_LIMIT, std::min(VOLTAGE_MOTOR_LIMIT, right));
  left = std::max(-1.0f * VOLTAGE_MOTOR_LIMIT, std::min(VOLTAGE_MOTOR_LIMIT, left));

  // モーター電圧を設定
  driver_->motor_right->speed(static_cast<int>(right * 1000.0f), sensor_->getSensed().battery_voltage);
  driver_->motor_left->speed(static_cast<int>(left * 1000.0f), sensor_->getSensed().battery_voltage);
}

// 動作完了待ち
void Motion::wait() {
  handle_notify_ = xTaskGetCurrentTaskHandle();
  ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
}
