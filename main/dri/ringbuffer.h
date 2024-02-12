#pragma once

#include <array>

namespace data {
template <typename T, std::size_t N>
class RingBuffer {
 private:
  // 先頭を指す添字
  std::size_t head_;
  // 末尾を指す添字
  std::size_t tail_;
  // 現在の要素数
  std::size_t size_;
  // 添字を最大要素数で切り捨てるマスク
  std::size_t mask_;
  // 要素を保持する配列
  std::array<T, N> buffer_;

 public:
  // コンストラクタ
  explicit RingBuffer() : head_(0), tail_(0), size_(0), buffer_() {
    static_assert(N && (N & (N - 1)) == 0, "N must be a power of 2.");
    mask_ = N - 1;
  }
  // デストラクタ
  ~RingBuffer() = default;

  // バッファをリセットする
  void reset() {
    head_ = 0;
    tail_ = 0;
    size_ = 0;
  }

  // 最大要素数を返す
  constexpr std::size_t max_size() { return N; }
  // 現在の要素数を返す
  std::size_t size() { return size_; }

  // 添字アクセス (読み)
  const T &operator[](std::size_t index) const {
    return buffer_[(head_ + index) & mask_];
  }
  // 添字アクセス (書き)
  T &operator[](std::size_t index) { return buffer_[(head_ + index) & mask_]; }

  // 先頭のデータを取得
  const T &front() { return buffer_[head_]; }
  // 先頭にデータを追加
  bool pushFront(T &data) {
    if (size_ == N) {
      return false;
    }
    size_++;
    head_ = (head_ - 1) & mask_;
    buffer_[head_] = data;
    return true;
  }
  // 先頭のデータを削除
  bool popFront() {
    if (size_ == 0) {
      return false;
    }
    head_ = (head_ + 1) & mask_;
    size_--;
    return true;
  }
  // 末尾のデータを取得
  const T &back() { return buffer_[tail_]; }
  // 末尾にデータを追加
  bool pushBack(T &data) {
    if (size_ == N) {
      return false;
    }
    size_++;
    buffer_[tail_] = data;
    tail_ = (tail_ + 1) & mask_;
    return true;
  }
  // 末尾のデータを削除
  bool popBack() {
    if (size_ == 0) {
      return false;
    }
    tail_ = (tail_ - 1) & mask_;
    size_--;
    return true;
  }
};
}  // namespace data