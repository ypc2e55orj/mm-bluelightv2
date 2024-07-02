#pragma once

// C++
#include <bitset>
#include <cmath>
#include <vector>

// ESP-IDF
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <rom/ets_sys.h>

// Project
#include "spi.h"

class Imu {
 public:
  static constexpr float ANGULAR_RATE_SENSITIVITY = 70.0f;          // [mdps/LSB]
  static constexpr float LINEAR_ACCELERATION_SENSITIVITY = 0.061f;  // [mg/LSB]

  template <typename T>
  struct Axis {
    T x;
    T y;
    T z;
  };

 private:
  Spi &spi_;
  int index_;
  uint8_t *rx_buffer_, *tx_buffer_;
  Axis<int16_t> raw_gyro_, raw_accel_;
  Axis<float> gyro_, accel_;

  // 送受信バッファサイズ
  static constexpr size_t BUFFER_SIZE = 12;

  // レジスタ
  static constexpr uint8_t REG_WHO_AM_I = 0x0F;
  static constexpr uint8_t DAT_WHO_AM_I = 0x6B;

  static constexpr uint8_t REG_CTRL1_XL = 0x10;
  static constexpr uint8_t BIT_CTRL1_XL_ODR_XL3 = 7;
  static constexpr uint8_t BIT_CTRL1_XL_ODR_XL2 = 6;
  static constexpr uint8_t BIT_CTRL1_XL_ODR_XL1 = 5;
  static constexpr uint8_t BIT_CTRL1_XL_ODR_XL0 = 4;
  static constexpr uint8_t BIT_CTRL1_XL_FS1_XL = 3;
  static constexpr uint8_t BIT_CTRL1_XL_FS0_XL = 2;
  static constexpr uint8_t BIT_CTRL1_XL_LPF2_XL_EN = 1;

  static constexpr uint8_t REG_CTRL2_G = 0x11;
  static constexpr uint8_t BIT_CTRL2_G_ODR_G3 = 7;
  static constexpr uint8_t BIT_CTRL2_G_ODR_G2 = 6;
  static constexpr uint8_t BIT_CTRL2_G_ODR_G1 = 5;
  static constexpr uint8_t BIT_CTRL2_G_ODR_G0 = 4;
  static constexpr uint8_t BIT_CTRL2_G_FS1_G = 3;
  static constexpr uint8_t BIT_CTRL2_G_FS0_G = 2;
  static constexpr uint8_t BIT_CTRL2_G_FS_125 = 1;
  static constexpr uint8_t BIT_CTRL2_G_FS_4000 = 0;

  static constexpr uint8_t REG_CTRL3_C = 0x12;
  static constexpr uint8_t BIT_CTRL3_C_BDU = 6;
  static constexpr uint8_t BIT_CTRL3_C_SW_RESET = 0;

  static constexpr uint8_t REG_CTRL4_C = 0x13;
  static constexpr uint8_t BIT_CTRL4_C_LPF1_SEL_G = 1;

  static constexpr uint8_t REG_CTRL6_C = 0x15;
  static constexpr uint8_t BIT_CTRL6_C_USR_OFF_W = 3;
  static constexpr uint8_t BIT_CTRL6_C_FTYPE_2 = 2;
  static constexpr uint8_t BIT_CTRL6_C_FTYPE_1 = 1;
  static constexpr uint8_t BIT_CTRL6_C_FTYPE_0 = 0;

  static constexpr uint8_t REG_CTRL7_G = 0x16;
  static constexpr uint8_t BIT_CTRL7_G_USR_OFF_ON_OUT = 1;

  static constexpr uint8_t REG_CTRL8_XL = 0x17;
  static constexpr uint8_t BIT_CTRL8_XL_HPCF_XL_2 = 7;
  static constexpr uint8_t BIT_CTRL8_XL_HPCF_XL_1 = 6;
  static constexpr uint8_t BIT_CTRL8_XL_HPCF_XL_0 = 5;
  static constexpr uint8_t BIT_CTRL8_XL_FASTSETTL_MODE_XL = 3;

  static constexpr uint8_t REG_CTRL9_XL = 0x18;
  static constexpr uint8_t BIT_CTRL9_XL_I3C_DISABLE = 1;

  static constexpr uint8_t REG_OUTX_L_G = 0x22;

  static constexpr uint8_t REG_X_OFS_USR = 0x73;
  static constexpr uint8_t REG_Y_OFS_USR = 0x74;
  static constexpr uint8_t REG_Z_OFS_USR = 0x75;

  /**
   * Accel X: -0.016088 x + -1.692139
   * Accel Y: -0.030933 x + -45.486130
   * Accel Z: -0.130897 x + 1004.997559
   * -1, -46, 5
   * Gyro X: 1.103957 x + 186.606155
   * Gyro Y: -1.892573 x + -283.435059
   * Gyro Z: -3.169824 x + 62.102707
   */
  static constexpr int8_t DAT_X_OFS_USR = -1;
  static constexpr int8_t DAT_Y_OFS_USR = -46;
  static constexpr int8_t DAT_Z_OFS_USR = 5;

  uint8_t read_byte(uint8_t reg) {
    auto trans = spi_.transaction(index_);
    uint8_t *p = trans->rx_data;
    trans->flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans->tx_buffer = nullptr;
    trans->rx_buffer = nullptr;
    trans->addr = reg | 0x80;
    trans->length = 8;
    trans->rxlength = 0;
    spi_.transmit(index_);
    return p[0];
  }
  bool write_byte(uint8_t reg, uint8_t data) {
    auto trans = spi_.transaction(index_);
    trans->flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans->tx_buffer = nullptr;
    trans->rx_buffer = nullptr;
    trans->addr = reg;
    trans->length = 8;
    trans->rxlength = 0;
    trans->tx_data[0] = data;
    bool ret = spi_.transmit(index_);
    return ret;
  }

 public:
  explicit Imu(Spi &spi, gpio_num_t spics_io_num) : spi_(spi), raw_gyro_(), raw_accel_(), gyro_(), accel_() {
    // 転送用バッファを確保
    tx_buffer_ = reinterpret_cast<uint8_t *>(heap_caps_calloc(BUFFER_SIZE, sizeof(uint8_t), MALLOC_CAP_DMA));
    rx_buffer_ = reinterpret_cast<uint8_t *>(heap_caps_calloc(BUFFER_SIZE, sizeof(uint8_t), MALLOC_CAP_DMA));

    // バスにデバイスを追加
    index_ = spi_.add(0, 8, 3, SPI_MASTER_FREQ_10M, spics_io_num, 1);

    std::bitset<8> reg;
    // 相手が合っているか確認
    assert(read_byte(REG_WHO_AM_I) == DAT_WHO_AM_I);
    reg = read_byte(REG_CTRL3_C);
    // ソフトウェア・リセット
    reg[BIT_CTRL3_C_SW_RESET] = true;
    // リセット実行
    write_byte(REG_CTRL3_C, static_cast<uint8_t>(reg.to_ulong()));

    // 初期設定
    reg = read_byte(REG_CTRL9_XL);
    // I3Cを無効化
    reg[BIT_CTRL9_XL_I3C_DISABLE] = true;
    // CTRL9_XLを反映
    write_byte(REG_CTRL9_XL, static_cast<uint8_t>(reg.to_ulong()));
    reg = read_byte(REG_CTRL3_C);
    // 読み出ししているレジスタは更新しない(Block Data Update)
    reg[BIT_CTRL3_C_BDU] = true;
    // CTRL3_Cを反映
    write_byte(REG_CTRL3_C, static_cast<uint8_t>(reg.to_ulong()));

    // 加速度計の設定
    reg = read_byte(REG_CTRL1_XL);
    // 出力レートを1.66kHzに設定
    reg[BIT_CTRL1_XL_ODR_XL3] = true;
    reg[BIT_CTRL1_XL_ODR_XL2] = false;
    reg[BIT_CTRL1_XL_ODR_XL1] = false;
    reg[BIT_CTRL1_XL_ODR_XL0] = false;
    // スケールを+-2gに設定
    reg[BIT_CTRL1_XL_FS1_XL] = false;
    reg[BIT_CTRL1_XL_FS0_XL] = false;
    // LPF2を有効
    reg[BIT_CTRL1_XL_LPF2_XL_EN] = true;
    // CTRL1_XLを反映
    write_byte(REG_CTRL1_XL, static_cast<uint8_t>(reg.to_ulong()));
    reg = read_byte(REG_CTRL8_XL);
    // フィルタをLow pass, ODR/10に設定
    reg[BIT_CTRL8_XL_HPCF_XL_2] = false;
    reg[BIT_CTRL8_XL_HPCF_XL_1] = false;
    reg[BIT_CTRL8_XL_HPCF_XL_0] = true;
    reg[BIT_CTRL8_XL_FASTSETTL_MODE_XL] = true;
    // CTRL8_XLを反映
    write_byte(REG_CTRL8_XL, static_cast<uint8_t>(reg.to_ulong()));
    reg = read_byte(REG_CTRL6_C);
    // オフセットの重みを2^-10 g/LSBに設定
    reg[BIT_CTRL6_C_USR_OFF_W] = false;
    // CTRL6_Cを反映
    write_byte(REG_CTRL6_C, static_cast<uint8_t>(reg.to_ulong()));
    reg = read_byte(REG_CTRL7_G);
    // オフセットを有効
    reg[BIT_CTRL7_G_USR_OFF_ON_OUT] = true;
    // CTRL7_Gを反映
    write_byte(REG_CTRL7_G, static_cast<uint8_t>(reg.to_ulong()));
    write_byte(REG_X_OFS_USR, static_cast<uint8_t>(DAT_X_OFS_USR));
    write_byte(REG_Y_OFS_USR, static_cast<uint8_t>(DAT_Y_OFS_USR));
    write_byte(REG_Z_OFS_USR, static_cast<uint8_t>(DAT_Z_OFS_USR));

    // 角速度計の設定
    reg = read_byte(REG_CTRL2_G);
    // 出力レートを1.66kHzに設定
    reg[BIT_CTRL2_G_ODR_G3] = true;
    reg[BIT_CTRL2_G_ODR_G2] = false;
    reg[BIT_CTRL2_G_ODR_G1] = false;
    reg[BIT_CTRL2_G_ODR_G0] = false;
    // スケールを+-2000dpsに設定
    reg[BIT_CTRL2_G_FS1_G] = true;
    reg[BIT_CTRL2_G_FS0_G] = true;
    reg[BIT_CTRL2_G_FS_125] = false;
    reg[BIT_CTRL2_G_FS_4000] = false;
    // CTRL2_Gを反映
    write_byte(REG_CTRL2_G, static_cast<uint8_t>(reg.to_ulong()));
    reg = read_byte(REG_CTRL4_C);
    // LPF1を有効
    reg[BIT_CTRL4_C_LPF1_SEL_G] = true;
    // CTRL4_Cを反映
    write_byte(REG_CTRL4_C, static_cast<uint8_t>(reg.to_ulong()));
    reg = read_byte(REG_CTRL6_C);
    reg[BIT_CTRL6_C_FTYPE_2] = false;
    reg[BIT_CTRL6_C_FTYPE_1] = true;
    reg[BIT_CTRL6_C_FTYPE_0] = false;
    // CTRL6_Cを反映
    write_byte(REG_CTRL6_C, static_cast<uint8_t>(reg.to_ulong()));
  }
  ~Imu() {
    free(tx_buffer_);
    free(rx_buffer_);
  }

  bool update() {
    auto trans = spi_.transaction(index_);
    trans->flags = 0;
    trans->tx_buffer = tx_buffer_;
    trans->rx_buffer = rx_buffer_;
    trans->addr = REG_OUTX_L_G | 0x80;
    trans->length = 12 * 8;  // OUTX_L_G(22h) ~ OUTZ_H_A(2Dh)
    trans->rxlength = trans->length;
    bool ret = spi_.transmit(index_);
    if (ret) {
      auto res = reinterpret_cast<int16_t *>(rx_buffer_);
      raw_gyro_.x = res[0];
      raw_gyro_.y = res[1];
      raw_gyro_.z = res[2];
      raw_accel_.x = res[3];
      raw_accel_.y = res[4];
      raw_accel_.z = res[5];

      gyro_.x = static_cast<float>(raw_gyro_.x) * ANGULAR_RATE_SENSITIVITY;
      gyro_.y = static_cast<float>(raw_gyro_.y) * ANGULAR_RATE_SENSITIVITY;
      gyro_.z = static_cast<float>(raw_gyro_.z) * ANGULAR_RATE_SENSITIVITY;
      accel_.x = static_cast<float>(raw_accel_.x) * LINEAR_ACCELERATION_SENSITIVITY;
      accel_.y = static_cast<float>(raw_accel_.y) * LINEAR_ACCELERATION_SENSITIVITY;
      accel_.z = static_cast<float>(raw_accel_.z) * LINEAR_ACCELERATION_SENSITIVITY;
    }
    return ret;
  }

  const Axis<int16_t> &raw_angular_rate() { return raw_gyro_; }
  const Axis<int16_t> &raw_linear_acceleration() { return raw_accel_; }
  const Axis<float> &angular_rate() { return gyro_; }
  const Axis<float> &linear_acceleration() { return accel_; }

  void offset(int n) {
    const float output_data_rate = 1660;                                // [Hz]
    const float user_offset_weight = 1000.0f * std::pow(2.0f, -10.0f);  // [mg/LSB]
    const float delay = 1.0f / output_data_rate;
    float sum_x = 0.0f, sum_x_2 = 0.0f;
    Axis<float> accel_coeff{}, accel_inter{}, gyro_coeff{}, gyro_inter{};
    Axis<float> sum_accel_xy{}, sum_accel_y{}, sum_gyro_xy{}, sum_gyro_y{};
    Axis<int8_t> accel_offset{};

    // 現在の設定をクリア
    write_byte(REG_X_OFS_USR, 0);
    write_byte(REG_Y_OFS_USR, 0);
    write_byte(REG_Z_OFS_USR, 0);

    for (int i = 0; i < n; i++) {
      ets_delay_us(static_cast<uint32_t>(delay * 1000'000.0f));
      update();
      const float x = delay * static_cast<float>(i);
      auto &accel = raw_linear_acceleration();
      auto accel_x = static_cast<float>(accel.x) * LINEAR_ACCELERATION_SENSITIVITY;
      auto accel_y = static_cast<float>(accel.y) * LINEAR_ACCELERATION_SENSITIVITY;
      auto accel_z = static_cast<float>(accel.z) * LINEAR_ACCELERATION_SENSITIVITY;

      auto &gyro = raw_angular_rate();
      auto gyro_x = static_cast<float>(gyro.x) * ANGULAR_RATE_SENSITIVITY;
      auto gyro_y = static_cast<float>(gyro.y) * ANGULAR_RATE_SENSITIVITY;
      auto gyro_z = static_cast<float>(gyro.z) * ANGULAR_RATE_SENSITIVITY;

      sum_accel_y.x += accel_x;
      sum_accel_y.y += accel_y;
      sum_accel_y.z += accel_z;
      sum_accel_xy.x += x * static_cast<float>(accel_x);
      sum_accel_xy.y += x * static_cast<float>(accel_y);
      sum_accel_xy.z += x * static_cast<float>(accel_z);
      sum_gyro_y.x += static_cast<float>(gyro_x);
      sum_gyro_y.y += static_cast<float>(gyro_y);
      sum_gyro_y.z += static_cast<float>(gyro_z);
      sum_gyro_xy.x += x * static_cast<float>(gyro_x);
      sum_gyro_xy.y += x * static_cast<float>(gyro_y);
      sum_gyro_xy.z += x * static_cast<float>(gyro_z);
      sum_x += x;
      sum_x_2 += std::pow(x, 2.0f);
    }
    auto coeff_a = [&](float sum_xy, float sum_y) {
      return (static_cast<float>(n) * sum_xy - sum_x * sum_y) /
             (static_cast<float>(n) * sum_x_2 - std::pow(sum_x, 2.0f));
    };
    auto inter_b = [&](float sum_xy, float sum_y) {
      return (sum_x_2 * sum_y - sum_xy * sum_x) / (static_cast<float>(n) * sum_x_2 - std::pow(sum_x, 2.0f));
    };
    accel_coeff.x = coeff_a(sum_accel_xy.x, sum_accel_y.x);
    accel_coeff.y = coeff_a(sum_accel_xy.y, sum_accel_y.y);
    accel_coeff.z = coeff_a(sum_accel_xy.z, sum_accel_y.z);
    accel_inter.x = inter_b(sum_accel_xy.x, sum_accel_y.x);
    accel_inter.y = inter_b(sum_accel_xy.y, sum_accel_y.y);
    accel_inter.z = inter_b(sum_accel_xy.z, sum_accel_y.z);
    accel_offset.x = static_cast<int8_t>(accel_inter.x / user_offset_weight);
    accel_offset.y = static_cast<int8_t>(accel_inter.y / user_offset_weight);
    accel_offset.z = static_cast<int8_t>((accel_inter.z - 1000.0f) / user_offset_weight);
    write_byte(REG_X_OFS_USR, static_cast<uint8_t>(accel_offset.x));
    write_byte(REG_Y_OFS_USR, static_cast<uint8_t>(accel_offset.y));
    write_byte(REG_Z_OFS_USR, static_cast<uint8_t>(accel_offset.z));

    gyro_coeff.x = coeff_a(sum_gyro_xy.x, sum_gyro_y.x);
    gyro_coeff.y = coeff_a(sum_gyro_xy.y, sum_gyro_y.y);
    gyro_coeff.z = coeff_a(sum_gyro_xy.z, sum_gyro_y.z);
    gyro_inter.x = inter_b(sum_gyro_xy.x, sum_gyro_y.x);
    gyro_inter.y = inter_b(sum_gyro_xy.y, sum_gyro_y.y);
    gyro_inter.z = inter_b(sum_gyro_xy.z, sum_gyro_y.z);

    printf("Accel X: %f x + %f\n", static_cast<double>(accel_coeff.x), static_cast<double>(accel_inter.x));
    printf("Accel Y: %f x + %f\n", static_cast<double>(accel_coeff.y), static_cast<double>(accel_inter.y));
    printf("Accel Z: %f x + %f\n", static_cast<double>(accel_coeff.z), static_cast<double>(accel_inter.z));
    printf("%d, %d, %d\n", accel_offset.x, accel_offset.y, accel_offset.z);

    printf("Gyro X: %f x + %f\n", static_cast<double>(gyro_coeff.x), static_cast<double>(gyro_inter.x));
    printf("Gyro Y: %f x + %f\n", static_cast<double>(gyro_coeff.y), static_cast<double>(gyro_inter.y));
    printf("Gyro Z: %f x + %f\n", static_cast<double>(gyro_coeff.z), static_cast<double>(gyro_inter.z));
  }

  void calibration() { offset(10000); }
};
