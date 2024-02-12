#pragma once

// C++
#include <cmath>

// ESP-IDF
#include <driver/mcpwm_prelude.h>

/**
 * 参考:
 * https://github.com/espressif/idf-extra-components/tree/458086bb459f6e37a3a5eba203ba75a70cece59f/bdc_motor
 */
class Motor {
 private:
  static constexpr uint32_t MCPWM_TIMER_RESOLUTION_HZ = 80'000'000;
  static constexpr uint32_t MCPWM_PWM_FREQUENCY_HZ = 100'000;
  static constexpr uint32_t MCPWM_TIMER_PERIOD_TICKS =
      MCPWM_TIMER_RESOLUTION_HZ / MCPWM_PWM_FREQUENCY_HZ;

  mcpwm_timer_handle_t timer_;
  mcpwm_oper_handle_t operator_;

  struct {
    mcpwm_cmpr_handle_t a, b;
  } comparator_;
  struct {
    mcpwm_gen_handle_t a, b;
  } generator_;

  int motor_voltage_;

 public:
  explicit Motor(int mcpwm_timer_group_id, gpio_num_t a_num, gpio_num_t b_num)
      : timer_(), operator_(), comparator_(), generator_(), motor_voltage_() {
    // タイマーを初期化
    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = mcpwm_timer_group_id;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = MCPWM_TIMER_RESOLUTION_HZ;
    timer_config.period_ticks = MCPWM_TIMER_PERIOD_TICKS;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_));

    mcpwm_operator_config_t operator_config = {};
    operator_config.group_id = mcpwm_timer_group_id;
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator_));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_, timer_));

    mcpwm_comparator_config_t comparator_config = {};
    comparator_config.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(
        mcpwm_new_comparator(operator_, &comparator_config, &comparator_.a));
    ESP_ERROR_CHECK(
        mcpwm_new_comparator(operator_, &comparator_config, &comparator_.b));
    mcpwm_comparator_set_compare_value(comparator_.a, 0);
    mcpwm_comparator_set_compare_value(comparator_.b, 0);

    mcpwm_generator_config_t generator_config = {};
    generator_config.gen_gpio_num = a_num;
    ESP_ERROR_CHECK(
        mcpwm_new_generator(operator_, &generator_config, &generator_.a));
    generator_config.gen_gpio_num = b_num;
    ESP_ERROR_CHECK(
        mcpwm_new_generator(operator_, &generator_config, &generator_.b));

    // https://docs.espressif.com/projects/esp-idf/en/v5.1.1/esp32s3/api-reference/peripherals/mcpwm.html#asymmetric-single-edge-active-high
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(
        generator_.a,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(
        generator_.a,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_.a,
                                       MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(
        generator_.b,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(
        generator_.b,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_.b,
                                       MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));
#pragma GCC diagnostic pop
  }
  ~Motor() {
    mcpwm_del_generator(generator_.a);
    mcpwm_del_generator(generator_.b);

    mcpwm_del_comparator(comparator_.a);
    mcpwm_del_comparator(comparator_.b);

    mcpwm_del_operator(operator_);

    mcpwm_del_timer(timer_);
  }

  bool enable() {
    esp_err_t enable_err = mcpwm_timer_enable(timer_);
    esp_err_t start_err =
        mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP);
    return enable_err == ESP_OK && start_err == ESP_OK;
  }
  bool disable() {
    esp_err_t stop_err =
        mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_STOP_EMPTY);
    esp_err_t disable_err = mcpwm_timer_disable(timer_);
    return stop_err == ESP_OK && disable_err == ESP_OK;
  }

  void brake() const {
    mcpwm_generator_set_force_level(generator_.a, 1, true);
    mcpwm_generator_set_force_level(generator_.b, 1, true);
  }
  void coast() const {
    mcpwm_generator_set_force_level(generator_.a, 0, true);
    mcpwm_generator_set_force_level(generator_.b, 0, true);
  }
  void speed(int motor_voltage, int battery_voltage) {
    motor_voltage_ = motor_voltage;
    auto duty =
        static_cast<float>(motor_voltage) / static_cast<float>(battery_voltage);
    mcpwm_generator_set_force_level(generator_.a, duty < 0.0f ? 0 : -1, true);
    mcpwm_generator_set_force_level(generator_.b, duty < 0.0f ? -1 : 0, true);
    auto duty_ticks =
        static_cast<uint32_t>(MCPWM_TIMER_PERIOD_TICKS * std::abs(duty));
    mcpwm_comparator_set_compare_value(comparator_.a, duty_ticks);
    mcpwm_comparator_set_compare_value(comparator_.b, duty_ticks);
  }

  [[nodiscard]] int voltage() const { return motor_voltage_; }
};
