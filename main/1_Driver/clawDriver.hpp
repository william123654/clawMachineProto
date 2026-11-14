#pragma once

#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_err.h"

namespace Driver {

    class clawDriver {
        gpio_num_t enaPin;
        gpio_num_t dirPin;
        gpio_num_t pulsePin;

        static constexpr size_t MAX_STEPS = 500;
        static constexpr uint32_t MIN_HALF_US = 3;
        static constexpr uint32_t MAX_HALF_US = 32767;

        rmt_channel_handle_t channel;
        rmt_encoder_handle_t copyEnc;
        rmt_symbol_word_t symbols[MAX_STEPS]{};
        bool busy = false;

        public:
            clawDriver(gpio_num_t, gpio_num_t, gpio_num_t);
    }
}