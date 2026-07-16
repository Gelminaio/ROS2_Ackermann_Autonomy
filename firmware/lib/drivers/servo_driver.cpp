#include "servo_driver.h"

#include "config.h"

namespace drivers
{

    // Period of a 50 Hz PWM signal in microseconds.
    static constexpr uint32_t SERVO_PERIOD_US = 1000000UL / SERVO_PWM_FREQ_HZ;      // 20000 us
    static constexpr uint32_t SERVO_DUTY_MAX = (1UL << SERVO_PWM_RESOLUTION) - 1UL; // 65535

    bool ServoDriver::begin()
    {
        // Configure LEDC channel for the servo.
        // ledcSetup(channel, freq_hz, resolution_bits) → returns the actual frequency.
        double actual_freq = ledcSetup(
            SERVO_LEDC_CHANNEL,
            SERVO_PWM_FREQ_HZ,
            SERVO_PWM_RESOLUTION);

        if (actual_freq == 0)
        {
            LOGLN("[servo] ERROR: ledcSetup failed");
            return false;
        }

        ledcAttachPin(PIN_SERVO_STEERING, SERVO_LEDC_CHANNEL);

        // Park at center on startup so the wheels do not snap to a random position.
        setAngle(0.0f);

        initialized_ = true;

        LOGLN("[servo] initialized on GPIO %d, LEDC ch %d, %u Hz, %u-bit\n",
              PIN_SERVO_STEERING, SERVO_LEDC_CHANNEL,
              SERVO_PWM_FREQ_HZ, SERVO_PWM_RESOLUTION);
        LOGLN("[servo] pulse calibration: %u / %u / %u us (left/center/right)\n",
              SERVO_PULSE_LEFT_US, SERVO_PULSE_CENTER_US, SERVO_PULSE_RIGHT_US);

        return true;
    }

    float ServoDriver::setAngle(float angle_deg)
    {
        // Hard clamp to Ackermann mechanical limits.
        if (angle_deg > SERVO_ANGLE_MAX_DEG)
            angle_deg = SERVO_ANGLE_MAX_DEG;
        if (angle_deg < SERVO_ANGLE_MIN_DEG)
            angle_deg = SERVO_ANGLE_MIN_DEG;

        const uint32_t pulse_us = angleToPulseUs(angle_deg * SERVO_DIRECTION); // mirror
        const uint32_t duty = pulseUsToDuty(pulse_us);

        ledcWrite(SERVO_LEDC_CHANNEL, duty);

        current_angle_deg_ = angle_deg;
        return angle_deg;
    }

    uint32_t ServoDriver::pulseUsToDuty(uint32_t pulse_us)
    {
        // duty = pulse_us * (2^resolution - 1) / period_us
        return (pulse_us * SERVO_DUTY_MAX) / SERVO_PERIOD_US;
    }

    uint32_t ServoDriver::angleToPulseUs(float angle_deg)
    {
        // Piecewise-linear interpolation:
        //   negative angle → between LEFT and CENTER
        //   positive angle → between CENTER and RIGHT
        if (angle_deg >= 0.0f)
        {
            const float frac = angle_deg / SERVO_ANGLE_MAX_DEG; // 0..1
            return SERVO_PULSE_CENTER_US +
                   static_cast<uint32_t>(frac * (SERVO_PULSE_RIGHT_US - SERVO_PULSE_CENTER_US));
        }
        else
        {
            const float frac = -angle_deg / SERVO_ANGLE_MAX_DEG; // 0..1
            return SERVO_PULSE_CENTER_US -
                   static_cast<uint32_t>(frac * (SERVO_PULSE_CENTER_US - SERVO_PULSE_LEFT_US));
        }
    }

} // namespace drivers