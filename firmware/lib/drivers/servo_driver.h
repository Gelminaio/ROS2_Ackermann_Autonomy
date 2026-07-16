#pragma once

#include <Arduino.h>

namespace drivers
{

    // ============================================================================
    // Hardware-PWM servo driver using ESP32 LEDC peripheral.
    //
    // The servo accepts a 50 Hz PWM signal where the pulse width (typically
    // 1000-2000 us) maps to the shaft angle. We use 16-bit resolution for
    // fine-grained angular positioning.
    // ============================================================================
    class ServoDriver
    {
    public:
        // Initialize LEDC channel and attach to the steering GPIO.
        // Returns false if setup fails. Call once in setup().
        bool begin();

        // Set steering angle in degrees.
        // Input is clamped to [SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG].
        // Returns the actual (possibly clamped) angle commanded.
        float setAngle(float angle_deg);

        // Return the last commanded angle (after clamping).
        float getAngle() const { return current_angle_deg_; }

    private:
        // Convert pulse width in microseconds to LEDC duty ticks.
        static uint32_t pulseUsToDuty(uint32_t pulse_us);

        // Linear interpolation between calibrated extremes for a given angle.
        static uint32_t angleToPulseUs(float angle_deg);

        float current_angle_deg_ = 0.0f;
        bool initialized_ = false;
    };

} // namespace drivers