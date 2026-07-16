#pragma once

#include <Arduino.h>

namespace drivers
{

    class MotorDriver
    {
    public:
        MotorDriver(int pin_pwm_fwd,
                    int pin_pwm_rev,
                    int ledc_channel_fwd,
                    int ledc_channel_rev,
                    const char *label,
                    bool invert = false);

        bool begin();

        int16_t setDuty(int16_t duty);

        int16_t getDuty() const { return current_duty_; }

        // emergency stop: immediately set both PWMs to zero.
        void coast();

    private:
        enum class Direction
        {
            COAST,
            FORWARD,
            REVERSE
        };

        const int pin_fwd_;
        const int pin_rev_;
        const int ch_fwd_;
        const int ch_rev_;
        const char *label_;
        const bool invert_;

        int16_t current_duty_ = 0;
        Direction current_direction_ = Direction::COAST;
        bool initialized_ = false;
    };

}