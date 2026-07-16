#include "encoder_driver.h"

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace drivers
{

    static constexpr int16_t PCNT_H_LIM = 10000;
    static constexpr int16_t PCNT_L_LIM = -10000;

    EncoderDriver::EncoderDriver(int pin_a,
                                 int pin_b,
                                 int pcnt_unit,
                                 const char *label,
                                 bool invert)
        : pin_a_(pin_a),
          pin_b_(pin_b),
          unit_(static_cast<pcnt_unit_t>(pcnt_unit)),
          label_(label),
          invert_(invert) {}

    bool EncoderDriver::begin()
    {
        pcnt_config_t cfg = {};
        cfg.pulse_gpio_num = pin_a_;
        cfg.ctrl_gpio_num = pin_b_;
        cfg.lctrl_mode = PCNT_MODE_REVERSE;
        cfg.hctrl_mode = PCNT_MODE_KEEP;
        cfg.pos_mode = PCNT_COUNT_INC;
        cfg.neg_mode = PCNT_COUNT_DEC;
        cfg.counter_h_lim = PCNT_H_LIM;
        cfg.counter_l_lim = PCNT_L_LIM;
        cfg.unit = unit_;
        cfg.channel = PCNT_CHANNEL_0;

        esp_err_t err = pcnt_unit_config(&cfg);
        if (err != ESP_OK)
        {
            LOGLN("[encoder %s] ERROR: pcnt_unit_config failed (%d)\n",
                  label_, err);
            return false;
        }

        const uint16_t filter_cycles = (ENCODER_GLITCH_FILTER_NS * 80) / 1000;
        pcnt_set_filter_value(unit_, filter_cycles < 1023 ? filter_cycles : 1023);
        pcnt_filter_enable(unit_);

        pcnt_event_enable(unit_, PCNT_EVT_H_LIM);
        pcnt_event_enable(unit_, PCNT_EVT_L_LIM);

        static bool isr_service_installed = false;
        if (!isr_service_installed)
        {
            pcnt_isr_service_install(0);
            isr_service_installed = true;
        }
        pcnt_isr_handler_add(unit_, pcntOverflowHandler, this);

        pcnt_counter_pause(unit_);
        pcnt_counter_clear(unit_);
        pcnt_counter_resume(unit_);

        last_time_for_vel_ms_ = millis();
        initialized_ = true;

        LOGLN("[encoder %s] initialized on pins A=%d B=%d, PCNT unit %d, glitch filter %u ns\n",
              label_, pin_a_, pin_b_, static_cast<int>(unit_),
              ENCODER_GLITCH_FILTER_NS);
        return true;
    }

    void IRAM_ATTR EncoderDriver::pcntOverflowHandler(void *arg)
    {
        EncoderDriver *self = static_cast<EncoderDriver *>(arg);
        uint32_t status = 0;
        pcnt_get_event_status(self->unit_, &status);

        portENTER_CRITICAL_ISR(&self->mux_);
        if (status & PCNT_EVT_H_LIM)
        {
            self->overflow_count_ += PCNT_H_LIM;
        }
        if (status & PCNT_EVT_L_LIM)
        {
            self->overflow_count_ += PCNT_L_LIM;
        }
        portEXIT_CRITICAL_ISR(&self->mux_);
    }

    int32_t EncoderDriver::getCount()
    {
        int16_t hw_count = 0;
        int32_t overflow_snapshot = 0;

        // Disable interrupts briefly to read both counters atomically
        portENTER_CRITICAL(&mux_);
        pcnt_get_counter_value(unit_, &hw_count);
        overflow_snapshot = overflow_count_;
        portEXIT_CRITICAL(&mux_);

        const int32_t total = overflow_snapshot + static_cast<int32_t>(hw_count);
        return invert_ ? -total : total;
    }

    float EncoderDriver::getVelocityTicksPerSec()
    {
        const uint32_t now_ms = millis();
        const uint32_t dt_ms = now_ms - last_time_for_vel_ms_;

        if (dt_ms >= ENCODER_VELOCITY_WINDOW_MS)
        {
            const int32_t now_count = getCount();
            const int32_t dcount = now_count - last_count_for_vel_;
            cached_vel_tps_ = (1000.0f * static_cast<float>(dcount)) / static_cast<float>(dt_ms);

            last_count_for_vel_ = now_count;
            last_time_for_vel_ms_ = now_ms;
        }
        return cached_vel_tps_;
    }

    float EncoderDriver::getVelocityMps()
    {
        const float tps = getVelocityTicksPerSec();
        const float wheel_rad_per_sec =
            (tps * 2.0f * PI) / (static_cast<float>(ENCODER_TICKS_PER_REV) * GEAR_RATIO);
        return wheel_rad_per_sec * WHEEL_RADIUS_M;
    }

    float EncoderDriver::getVelocityMpsFiltered(float ema_alpha)
    {
        const uint32_t now_ms = millis();
        const int32_t now_count = getCount();

        const uint32_t dt_ms = now_ms - last_time_for_vel_ms_;
        if (dt_ms == 0)
        {
            const float wheel_rad_per_sec =
                (ema_velocity_tps_ * 2.0f * PI) /
                (static_cast<float>(ENCODER_TICKS_PER_REV) * GEAR_RATIO);
            return wheel_rad_per_sec * WHEEL_RADIUS_M;
        }

        const int32_t dcount = now_count - last_count_for_vel_;
        const float raw_tps = (1000.0f * static_cast<float>(dcount)) / static_cast<float>(dt_ms);

        // exponential moving average filter
        if (!ema_initialized_)
        {
            ema_velocity_tps_ = raw_tps;
            ema_initialized_ = true;
        }
        else
        {
            ema_velocity_tps_ = ema_alpha * raw_tps + (1.0f - ema_alpha) * ema_velocity_tps_;
        }

        last_count_for_vel_ = now_count;
        last_time_for_vel_ms_ = now_ms;

        // convert filtered ticks/s to m/s
        const float wheel_rad_per_sec =
            (ema_velocity_tps_ * 2.0f * PI) /
            (static_cast<float>(ENCODER_TICKS_PER_REV) * GEAR_RATIO);
        return wheel_rad_per_sec * WHEEL_RADIUS_M;
    }

    void EncoderDriver::resetCount()
    {
        portENTER_CRITICAL(&mux_);
        pcnt_counter_pause(unit_);
        pcnt_counter_clear(unit_);
        overflow_count_ = 0;
        last_count_for_vel_ = 0;
        last_time_for_vel_ms_ = millis();
        cached_vel_tps_ = 0.0f;
        pcnt_counter_resume(unit_);
        portEXIT_CRITICAL(&mux_);
    }

}