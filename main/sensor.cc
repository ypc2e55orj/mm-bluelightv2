#include "sensor.h"

// C++
#include <numbers>

// ESP-IDF
#include <esp_timer.h>

// コンストラクタ
Sensor::Sensor(Driver *dri) : dri_(dri), odom_(dri) {}
// コンストラクタ
Sensor::~Sensor() = default;

// 初期化
void Sensor::setup() {
  for (auto i = 0; i < WARM_UP_COUNTS; i++) {
    update();
  }
  reset();
}

// 更新
void Sensor::update() {
  dri_->battery->update();
  dri_->photo->update();
  dri_->imu->update();
  dri_->encoder_left->update();
  dri_->encoder_right->update();
  dri_->photo->wait();
  auto timestamp = esp_timer_get_time();
  odom_.update(timestamp - timestamp_);
  timestamp_ = timestamp;

  sensed_.velocity = odom_.velocity() / 1000;
  sensed_.length = odom_.length();
  sensed_.angular_velocity = odom_.angular_velocity();
  sensed_.angle = odom_.angle() * 180.0f / std::numbers::pi_v<float>;
  sensed_.battery_voltage = static_cast<float>(dri_->battery->average()) / 1000.0f;
}