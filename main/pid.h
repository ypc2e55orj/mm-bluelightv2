#pragma once

#include <cmath>

class Pid {
 private:
  struct Gain {
    struct {
      bool enable;
      float value;
    } kp, ki, kd;
  } gain_{};

  float prev_target_{};
  float prev_{};

  float sum_target_{};
  float sum_{};

 public:
  explicit Pid(float kp, float ki, float kd) {
    gain_.kp.enable = true;
    gain_.ki.enable = true;
    gain_.kd.enable = true;

    gain_.kp.value = kp;
    gain_.ki.value = ki;
    gain_.kd.value = kd;
    reset();
  }
  Gain &gain() { return gain_; }

  void reset() {
    prev_target_ = 0.0f;
    prev_ = 0.0f;
    sum_target_ = 0.0f;
    sum_ = 0.0f;
  }

  float update(float target, float current, float dt) {
    float ret = 0.0f;

    if (gain_.kp.enable) {
      ret += gain_.kp.value * (target - current);
    }
    if (gain_.ki.enable) {
      ret += gain_.ki.value * (sum_target_ - sum_);
    }
    if (gain_.kd.enable) {
      ret += gain_.kd.value * (prev_target_ - prev_) / dt;
    }

    if ((!std::signbit(target) && (sum_target_ + target) > sum_target_) ||
        (std::signbit(target) && (sum_target_ + target) < sum_target_)) {
      sum_target_ += target;
    }

    if ((!std::signbit(current) && (sum_ + current) > sum_) || (std::signbit(current) && (sum_ + current) < sum_)) {
      sum_ += current;
    }

    prev_target_ = target;
    prev_ = current;

    return ret;
  }
};