#pragma once

// #include "driver/adc.h"
#include "esp_adc/adc_continuous.h"

namespace adc_task {

void setup();

void dump_stats();

}  // namespace adc_task