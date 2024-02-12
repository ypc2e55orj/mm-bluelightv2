#pragma once

// C++
#include <memory>

// Project
#include "adc.h"
#include "battery.h"
#include "buzzer.h"
#include "encoder.h"
#include "fs.h"
#include "gpio.h"
#include "imu.h"
#include "indicator.h"
#include "motor.h"
#include "photo.h"
#include "pin.h"
#include "spi.h"

struct DriverClass {
  std::unique_ptr<Fs> fs;
  std::unique_ptr<Battery> battery;
  std::unique_ptr<Buzzer> buzzer;
  std::unique_ptr<Encoder> encoder_left;
  std::unique_ptr<Encoder> encoder_right;
  std::unique_ptr<Imu> imu;
  std::unique_ptr<Indicator> indicator;
  std::unique_ptr<Motor> motor_left;
  std::unique_ptr<Motor> motor_right;
  std::unique_ptr<Photo> photo;

  /**
   * @brief Core 0で使用されるドライバ
   * @details
   * Core 0 (Sensorタスク、Motionタスク)で使用されるドライバ
   */
  void init_pro() {
    // clang-format off
  battery = std::make_unique<Battery>(
      ADC_UNIT_BATTERY,
      ADC_CHANNEL_BATTERY);

  spi_imu_ = std::make_unique<Spi>(
      SPI3_HOST,
      GPIO_NUM_IMU_SPI_MISO,
      GPIO_NUM_IMU_SPI_MOSI,
      GPIO_NUM_IMU_SPI_SCLK,
      16);
  imu = std::make_unique<Imu>(
      *spi_imu_,
      GPIO_NUM_IMU_SPI_CS);

  spi_encoder_ = std::make_unique<Spi>(
      SPI2_HOST,
      GPIO_NUM_ENCODER_SPI_MISO,
      GPIO_NUM_ENCODER_SPI_MOSI,
      GPIO_NUM_ENCODER_SPI_SCLK,
      4);
  encoder_left = std::make_unique<Encoder>(
      *spi_encoder_,
      GPIO_NUM_ENCODER_SPI_CS_LEFT);
  encoder_right = std::make_unique<Encoder>(
      *spi_encoder_,
      GPIO_NUM_ENCODER_SPI_CS_RIGHT);

  motor_left = std::make_unique<Motor>(
      0,
      GPIO_NUM_MOTOR_LEFT_IN1,
      GPIO_NUM_MOTOR_LEFT_IN2);
  motor_right = std::make_unique<Motor>(
      1,
      GPIO_NUM_MOTOR_RIGHT_IN1,
      GPIO_NUM_MOTOR_RIGHT_IN2);

  Photo::Config config{
      .adc_unit = ADC_UNIT_PHOTO,
      .adc_channel = {
          ADC_CHANNEL_PHOTO_LEFT90,
          ADC_CHANNEL_PHOTO_LEFT45,
          ADC_CHANNEL_PHOTO_RIGHT45,
          ADC_CHANNEL_PHOTO_RIGHT90},
      .gpio_num = {
          GPIO_NUM_PHOTO_LEFT90,
          GPIO_NUM_PHOTO_LEFT45,
          GPIO_NUM_PHOTO_RIGHT45,
          GPIO_NUM_PHOTO_RIGHT90}};
  photo = std::make_unique<Photo>(config);
  // clang-format off
  }
/**
 * @brief Core 1で使用されるドライバ
 * @details
 * Core 1 (Mainタスク、Uiタスク)で使用されるドライバ
 */
void init_app() {
  // clang-format off
  fs = std::make_unique<Fs>(10);

  indicator = std::make_unique<Indicator>(
  GPIO_NUM_INDICATOR,
  NUM_INDICATORS, 4, true);

  buzzer = std::make_unique<Buzzer>(
    GPIO_NUM_BUZZER, 4, false);
    // clang-format on
  }

 private:
  std::unique_ptr<Spi> spi_encoder_;
  std::unique_ptr<Spi> spi_imu_;
};

extern DriverClass Driver;