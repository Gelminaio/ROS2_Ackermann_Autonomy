# ROS 2 Ackermann Autonomy Workspace

This Colcon workspace contains the autonomous driving stack for a custom-built Ackermann steering platform.

## Current State: Hardware Abstraction Layer (HAL)
Before introducing ROS 2 middleware, we developed a pure Python HAL (`ackermann_hal.py`) to interface with the low-level mechatronics, handling:
* Asynchronous I2C communication with the traction motor controller.
* Slew-rate limited PWM control for the steering servo.
* Software-based anti-backlash compensation and offset calibration.

## Directory Structure (Colcon-ready)
* `src/ackermann_core/hardware/`: Contains `ackermann_hal.py` (Base driver).
* `src/ackermann_core/tools/`: Contains `steering_calibration.py` (Bench tool for mechanical tuning).

*Next steps: Wrap the HAL into a `base_controller_node` and implement I2C encoder odometry.*
