#pragma once

#include "ringbuffer.h"

namespace data {
template <typename T>
concept Numeric = std::integral<T> || std::floating_point<T>;

template <Numeric T, Numeric U, std::size_t N>
class MovingAverage {
 private:
  // サンプルのリングバッファ
  RingBuffer<T, N> samples_;
  // 積算値のメモ
  U sums_;

 public:
  explicit MovingAverage() { reset(); }
  ~MovingAverage() = default;

  void reset() {
    samples_.reset();
    sums_ = static_cast<U>(0);
  }

  U update(T sample) {
    if (samples_.size() == 0) [[unlikely]] {
      sums_ = static_cast<U>(0);
      // 初回は与えられた値でバッファを満たす
      for (std::size_t n = 0; n < N; n++) {
        samples_.pushBack(sample);
        sums_ += static_cast<U>(sample);
      }
    } else {
      // サンプルのうち最も古いデータを取得して削除
      const T oldest = samples_.front();
      samples_.popFront();
      // 最も古いデータ分を除去
      sums_ -= oldest;
      sums_ += sample;
      samples_.pushBack(sample);
    }

    // 平均して返す
    return sums_ / static_cast<U>(N);
  }
};
}  // namespace data