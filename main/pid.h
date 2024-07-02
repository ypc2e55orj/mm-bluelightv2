#pragma once

#include <cmath>

class Pid {
 private:
  float gain_kp_{};
  float gain_ki_{};
  float gain_kd_{};

  // 前回の誤差量
  float prev_error_{};

  // 誤差の積分値
  float sum_error_{};

 public:
  explicit Pid(float kp, float ki, float kd) {
    gain_kp_ = kp;
    gain_ki_ = ki;
    gain_kd_ = kd;
    reset();
  }

  void reset() {
    prev_error_ = 0.0f;
    sum_error_ = 0.0f;
  }

  float update(float target, float current, float t) {
    // 積分値は台形近似
    // https://controlabo.com/pid-program/
    auto error = target - current;
    sum_error_ = (error + prev_error_) * t / 2.0f;
    auto ret = gain_kp_ * error + gain_ki_ * sum_error_ + gain_kd_ * (error - prev_error_) / t;
    prev_error_ = error;

    return ret;
  }
};
