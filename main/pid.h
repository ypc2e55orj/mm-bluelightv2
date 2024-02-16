#pragma once

#include <cmath>

class Pid {
 private:
  float gain_kp_{};
  float gain_ki_{};
  float gain_kd_{};

  float prev_target_{};
  float prev_{};

  float sum_target_{};
  float sum_{};

 public:
  explicit Pid(float kp, float ki, float kd) {
    gain_kp_ = kp;
    gain_ki_ = ki;
    gain_kd_ = kd;
    reset();
  }

  void reset() {
    prev_target_ = 0.0f;
    prev_ = 0.0f;
    sum_target_ = 0.0f;
    sum_ = 0.0f;
  }

  float update(float target, float current, float dt) {
    float ret = 0.0f;

    ret = gain_kp_ * (target - current) + gain_ki_ * (sum_target_ - sum_) - gain_kd_ * (prev_target_ - prev_) / dt;

    prev_target_ = target;
    prev_ = current;

    if ((!std::signbit(target) && (sum_target_ + target) > sum_target_) ||
        (std::signbit(target) && (sum_target_ + target) < sum_target_)) {
      sum_target_ += target;
    }
    if ((!std::signbit(current) && (sum_ + current) > sum_) || (std::signbit(current) && (sum_ + current) < sum_)) {
      sum_ += current;
    }

    return ret;
  }
};