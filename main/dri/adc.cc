#include "adc.h"

// 各ユニットのハンドラ
adc_oneshot_unit_handle_t Adc::unit1_ = nullptr;
adc_oneshot_unit_handle_t Adc::unit2_ = nullptr;
// 各ユニットの補正ハンドラ
adc_cali_handle_t Adc::unit1_cali_ = nullptr;
adc_cali_handle_t Adc::unit2_cali_ = nullptr;
