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

void Sensor::updateWallSensor(Sensed &sensed) {
  auto &left90 = dri_->photo->left90();
  auto &left45 = dri_->photo->left45();
  auto &right45 = dri_->photo->right45();
  auto &right90 = dri_->photo->right90();

  // TODO: 後で簡潔に書き直す
  // 左90度 (前壁)
  sensed.wall_left90.raw = left90.flash - left90.ambient;
  sensed.wall_left90.exist = sensed.wall_left90.raw > WALL_THRESHOLD_EXIST[0];
  sensed.wall_left90.error = sensed.wall_left90.exist ? sensed.wall_left90.raw - WALL_REFERENCE_VALUE[0] : 0;
  // 左45度 (左壁)
  sensed.wall_left45.raw = left45.flash - left45.ambient;
  sensed.wall_left45.exist = sensed.wall_left45.raw > WALL_THRESHOLD_EXIST[1];
  sensed.wall_left45.error = sensed.wall_left45.exist ? sensed.wall_left45.raw - WALL_REFERENCE_VALUE[1] : 0;
  // 右45度 (右壁)
  sensed.wall_right45.raw = right45.flash - right45.ambient;
  sensed.wall_right45.exist = sensed.wall_right45.raw > WALL_THRESHOLD_EXIST[2];
  sensed.wall_right45.error = sensed.wall_right45.exist ? sensed.wall_right45.raw - WALL_REFERENCE_VALUE[2] : 0;
  // 右90度 (前壁)
  sensed.wall_right90.raw = right90.flash - right90.ambient;
  sensed.wall_right90.exist = sensed.wall_right90.raw > WALL_THRESHOLD_EXIST[3];
  sensed.wall_right90.error = sensed.wall_right90.exist ? sensed.wall_right90.raw - WALL_REFERENCE_VALUE[3] : 0;
}

// 更新
void Sensor::update() {
  // 最新のセンサー値を取得
  dri_->battery->update();
  dri_->photo->update();
  dri_->imu->update();
  dri_->encoder_left->update();
  dri_->encoder_right->update();
  dri_->photo->wait();
  // オドメトリを計算
  auto timestamp = esp_timer_get_time();
  odom_.update(timestamp - timestamp_);
  timestamp_ = timestamp;

  // 値を設定
  Sensed sensed{};
  sensed.velocity = odom_.velocity() / 1000.0f;
  sensed.angular_velocity = odom_.angular_velocity();
  sensed.angle = odom_.angle() * 180.0f / std::numbers::pi_v<float>;
  sensed.length = odom_.length();
  sensed.x = odom_.x();
  sensed.y = odom_.y();
  sensed.battery_voltage = dri_->battery->voltage();
  sensed.battery_voltage_average = dri_->battery->average();
  updateWallSensor(sensed);

  // キューに上書き
  sensed_queue_.overwrite(&sensed);
}