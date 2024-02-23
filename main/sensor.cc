#include "sensor.h"

// C++
#include <numbers>

// ESP-IDF
#include <esp_timer.h>

// コンストラクタ
Sensor::Sensor(Driver *dri) : driver_(dri), odom_(dri) {}
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
  auto &right90 = driver_->photo->right90();
  auto &right45 = driver_->photo->right45();
  auto &left45 = driver_->photo->left45();
  auto &left90 = driver_->photo->left90();

  // TODO: 後で簡潔に書き直す

  // 右90度 (前壁)
  sensed.wall_right90.raw = right90.flash - right90.ambient;
  sensed.wall_right90.exist = sensed.wall_right90.raw > WALL_THRESHOLD_EXIST[PARAMETER_WALL_RIGHT90];
  sensed.wall_right90.error =
      sensed.wall_right90.exist ? sensed.wall_right90.raw - WALL_REFERENCE_VALUE[PARAMETER_WALL_RIGHT90] : 0;
  // 右45度 (右壁)
  sensed.wall_right45.raw = right45.flash - right45.ambient;
  sensed.wall_right45.exist = sensed.wall_right45.raw > WALL_THRESHOLD_EXIST[PARAMETER_WALL_RIGHT45];
  sensed.wall_right45.error =
      sensed.wall_right45.exist ? sensed.wall_right45.raw - WALL_REFERENCE_VALUE[PARAMETER_WALL_RIGHT45] : 0;
  // 左45度 (左壁)
  sensed.wall_left45.raw = left45.flash - left45.ambient;
  sensed.wall_left45.exist = sensed.wall_left45.raw > WALL_THRESHOLD_EXIST[PARAMETER_WALL_LEFT45];
  sensed.wall_left45.error =
      sensed.wall_left45.exist ? sensed.wall_left45.raw - WALL_REFERENCE_VALUE[PARAMETER_WALL_LEFT45] : 0;
  // 左90度 (前壁)
  sensed.wall_left90.raw = left90.flash - left90.ambient;
  sensed.wall_left90.exist = sensed.wall_left90.raw > WALL_THRESHOLD_EXIST[PARAMETER_WALL_LEFT90];
  sensed.wall_left90.error =
      sensed.wall_left90.exist ? sensed.wall_left90.raw - WALL_REFERENCE_VALUE[PARAMETER_WALL_LEFT90] : 0;
}

// 更新
void Sensor::update() {
  // 最新のセンサー値を取得
  driver_->battery->update();
  driver_->photo->update();
  driver_->imu->update();
  driver_->encoder_right->update();
  driver_->encoder_left->update();
  driver_->photo->wait();
  // オドメトリを計算
  auto timestamp = esp_timer_get_time();
  odom_.update(timestamp - timestamp_);
  timestamp_ = timestamp;

  // 値を設定
  sensed_.velocity = odom_.velocity() / 1000.0f;
  sensed_.angular_velocity = odom_.angular_velocity();
  sensed_.angle = odom_.angle() * 180.0f / std::numbers::pi_v<float>;
  sensed_.length = odom_.length();
  sensed_.x = odom_.x();
  sensed_.y = odom_.y();
  sensed_.battery_voltage = driver_->battery->voltage();
  sensed_.battery_voltage_average = driver_->battery->average();
  updateWallSensor(sensed_);
}
