#include <Arduino.h>

#include "globals.h"
#include "servo_driver.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "imu_driver.h"
#include "pid_controller.h"
#include "config.h"
#include "vehicle_state.h"

#include "motor_control_task.h"
#include "imu_task.h"
#include "comms_task.h"
#include "safety_task.h"
#include "telemetry_task.h"

#include "esp_task_wdt.h"

// global shared state, in future steps will be protected with mutex.
drivers::ServoDriver g_servo;
drivers::MotorDriver g_motor_left(PIN_MOTOR_L_PWM_FWD, PIN_MOTOR_L_PWM_REV, MOTOR_L_FWD_LEDC_CHANNEL, MOTOR_L_REV_LEDC_CHANNEL, "left", true);
drivers::MotorDriver g_motor_right(PIN_MOTOR_R_PWM_FWD, PIN_MOTOR_R_PWM_REV, MOTOR_R_FWD_LEDC_CHANNEL, MOTOR_R_REV_LEDC_CHANNEL, "right");
drivers::EncoderDriver g_encoder_left(PIN_ENCODER_L_A, PIN_ENCODER_L_B, ENCODER_L_PCNT_UNIT, "left", true);
drivers::EncoderDriver g_encoder_right(PIN_ENCODER_R_A, PIN_ENCODER_R_B, ENCODER_R_PCNT_UNIT, "right");
drivers::ImuDriver g_imu(BNO085_I2C_ADDRESS, I2C_FREQUENCY_HZ);
control::PIDController g_pid_left(PID_KP_INITIAL, PID_KI_INITIAL, PID_KD_INITIAL, PERIOD_MOTOR_CONTROL_MS / 1000.0f, PID_INTEGRAL_CLAMP, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
control::PIDController g_pid_right(PID_KP_INITIAL, PID_KI_INITIAL, PID_KD_INITIAL, PERIOD_MOTOR_CONTROL_MS / 1000.0f, PID_INTEGRAL_CLAMP, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
control::SafetySupervisor g_safety;
VehicleState g_vehicle_state;
VehicleCommand g_vehicle_command;

// Setup: hardware bring-up + task creation, then return forever.
void setup()
{
    Serial.begin(921600);
    esp_task_wdt_init(SAFETY_TWDT_TIMEOUT_S, true);
    delay(500);
    LOGLN("\n=== Adaptive MPC Ackermann — Firmware Boot ===");
    LOG("ESP-IDF: %s\n", esp_get_idf_version());
    LOG("Free heap: %u bytes\n", ESP.getFreeHeap());

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // Initialize hardware drivers
    // servo
    if (!g_servo.begin())
    {
        LOGLN("FATAL: servo driver init failed");
    }
    // motors left and right
    if (!g_motor_left.begin())
    {
        LOGLN("FATAL: left motor driver init failed");
    }
    if (!g_motor_right.begin())
    {
        LOGLN("FATAL: right motor driver init failed");
    }
    // encoders left and right
    if (!g_encoder_left.begin())
    {
        LOGLN("FATAL: left encoder init failed");
    }
    if (!g_encoder_right.begin())
    {
        LOGLN("FATAL: right encoder init failed");
    }
    // IMU
    if (!g_imu.begin())
    {
        LOGLN("FATAL: IMU init failed");
    }
    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(
        tasks::safety_task, "safety",
        STACK_SAFETY, nullptr,
        PRIO_SAFETY, nullptr,
        CORE_CONTROL);

    xTaskCreatePinnedToCore(
        tasks::motor_control_task, "motor_ctrl",
        STACK_MOTOR_CONTROL, nullptr,
        PRIO_MOTOR_CONTROL, nullptr,
        CORE_CONTROL);

    xTaskCreatePinnedToCore(
        tasks::imu_task, "imu",
        STACK_IMU, nullptr,
        PRIO_IMU, nullptr,
        CORE_CONTROL);

    xTaskCreatePinnedToCore(
        tasks::comms_task, "comms",
        STACK_COMMS, nullptr,
        PRIO_COMMS, nullptr,
        CORE_COMMS);

    xTaskCreatePinnedToCore(
        tasks::telemetry_task, "telemetry",
        STACK_TELEMETRY, nullptr,
        PRIO_TELEMETRY, nullptr,
        CORE_COMMS);

    LOGLN("All tasks created. Entering idle.\n");
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}