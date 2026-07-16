#include "motor_driver.h"

#include "config.h"

namespace drivers
{

    MotorDriver::MotorDriver(int pin_pwm_fwd,
                             int pin_pwm_rev,
                             int ledc_channel_fwd,
                             int ledc_channel_rev,
                             const char *label,
                             bool invert)
        : pin_fwd_(pin_pwm_fwd),
          pin_rev_(pin_pwm_rev),
          ch_fwd_(ledc_channel_fwd),
          ch_rev_(ledc_channel_rev),
          label_(label),
          invert_(invert) {}

    bool MotorDriver::begin()
    {
        const double f1 = ledcSetup(ch_fwd_, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION);
        const double f2 = ledcSetup(ch_rev_, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION);
        if (f1 == 0 || f2 == 0)
        {
            LOGLN("[motor %s] ERROR: ledcSetup failed\n", label_);
            return false;
        }

        ledcAttachPin(pin_fwd_, ch_fwd_);
        ledcAttachPin(pin_rev_, ch_rev_);

        coast();

        initialized_ = true;
        LOGLN("[motor %s] initialized on pins fwd=%d rev=%d, ch %d/%d, %u Hz, %u-bit\n",
              label_, pin_fwd_, pin_rev_, ch_fwd_, ch_rev_,
              MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION);
        return true;
    }

    int16_t MotorDriver::setDuty(int16_t duty)
    {
        if (duty > MOTOR_DUTY_MAX)
            duty = MOTOR_DUTY_MAX;
        if (duty < MOTOR_DUTY_MIN)
            duty = MOTOR_DUTY_MIN;

        const int16_t effective_duty = invert_ ? -duty : duty;

        Direction new_direction;
        if (effective_duty == 0)
            new_direction = Direction::COAST;
        else if (effective_duty > 0)
            new_direction = Direction::FORWARD;
        else
            new_direction = Direction::REVERSE;

        const bool reversing =
            (current_direction_ == Direction::FORWARD && new_direction == Direction::REVERSE) ||
            (current_direction_ == Direction::REVERSE && new_direction == Direction::FORWARD);

        if (reversing)
        {
            ledcWrite(ch_fwd_, 0);
            ledcWrite(ch_rev_, 0);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_DEAD_TIME_MS));
        }

        switch (new_direction)
        {
        case Direction::FORWARD:
            ledcWrite(ch_rev_, 0);
            ledcWrite(ch_fwd_, static_cast<uint32_t>(effective_duty));
            break;
        case Direction::REVERSE:
            ledcWrite(ch_fwd_, 0);
            ledcWrite(ch_rev_, static_cast<uint32_t>(-effective_duty));
            break;
        case Direction::COAST:
        default:
            ledcWrite(ch_fwd_, 0);
            ledcWrite(ch_rev_, 0);
            break;
        }

        current_duty_ = duty;
        current_direction_ = new_direction;
        return duty;
    }

    void MotorDriver::coast()
    {
        ledcWrite(ch_fwd_, 0);
        ledcWrite(ch_rev_, 0);
        current_duty_ = 0;
        current_direction_ = Direction::COAST;
    }

}