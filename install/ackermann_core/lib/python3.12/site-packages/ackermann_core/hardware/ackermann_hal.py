import smbus2
import pigpio
import time
import sys
import tty
import termios
import select

# Hardware Configuration
I2C_ADDR = 0x34
MOTOR_REG = 51
ENCODER_LEFT_REG = 60
ENCODER_RIGHT_REG = 68
STEERING_PIN = 12

# Calibration Parameters
CENTER_PWM = 1475
LEFT_KICK = 30
RIGHT_KICK = -100
TICKS_PER_METER = 6270.0
LOOP_PERIOD_S = 0.02
STEER_STEP_PWM = 500
KICK_HOLD_S = 0.08
MAX_STEERING_OFFSET = 350

current_pwm = CENTER_PWM
steer_target_pwm = CENTER_PWM
steer_primary_target = CENTER_PWM
steer_apply_kick = False
steer_phase = "idle"
kick_hold_until = 0.0
desired_speed = 0
desired_steering_offset = 0
_last_odom_ticks = None
_last_odom_time = None

pi = pigpio.pi()
bus = smbus2.SMBus(1)

if not pi.connected:
    print("ERROR: pigpiod daemon not running. Execute 'sudo pigpiod' first.")
    sys.exit()

# Initialize steering position
pi.set_servo_pulsewidth(STEERING_PIN, CENTER_PWM)
time.sleep(0.5)

def _move_pwm_towards(target_pwm):
    global current_pwm
    if current_pwm == target_pwm:
        return True

    delta = target_pwm - current_pwm
    if abs(delta) <= STEER_STEP_PWM:
        current_pwm = target_pwm
    else:
        current_pwm += STEER_STEP_PWM if delta > 0 else -STEER_STEP_PWM

    pi.set_servo_pulsewidth(STEERING_PIN, current_pwm)
    return current_pwm == target_pwm

def set_robot(speed, steering_offset, now=None):
    global steer_target_pwm
    global steer_primary_target
    global steer_apply_kick
    global steer_phase
    global kick_hold_until

    if now is None:
        now = time.monotonic()
    
    # 1. Drive Control (I2C)
    def to_byte(v): return v if v >= 0 else (256 + v)
    
    vel_left = -speed
    vel_right = speed
    packet = [to_byte(vel_left), to_byte(vel_left), to_byte(vel_right), to_byte(vel_right)]
    
    try:
        bus.write_i2c_block_data(I2C_ADDR, MOTOR_REG, packet)
    except Exception:
        pass

    # 2. Steering Control (PWM), progressed one step per main-loop iteration
    target_pwm = CENTER_PWM + steering_offset
    target_pwm = max(1000, min(2000, target_pwm))

    if target_pwm != steer_target_pwm:
        steer_target_pwm = target_pwm

        # Backlash compensation logic preserved from original behavior.
        steer_apply_kick = target_pwm == CENTER_PWM and abs(current_pwm - CENTER_PWM) > 50
        kick_val = LEFT_KICK if current_pwm < CENTER_PWM else RIGHT_KICK
        steer_primary_target = (CENTER_PWM + kick_val) if steer_apply_kick else target_pwm

        if steer_primary_target != current_pwm:
            steer_phase = "primary"
        elif steer_apply_kick:
            steer_phase = "kick_wait"
            kick_hold_until = now + KICK_HOLD_S
        else:
            steer_phase = "idle"

    if steer_phase == "primary":
        if _move_pwm_towards(steer_primary_target):
            if steer_apply_kick:
                steer_phase = "kick_wait"
                kick_hold_until = now + KICK_HOLD_S
            else:
                steer_phase = "idle"
    elif steer_phase == "kick_wait":
        if now >= kick_hold_until:
            if steer_target_pwm != current_pwm:
                steer_phase = "post_kick"
            else:
                steer_phase = "idle"
    elif steer_phase == "post_kick":
        if _move_pwm_towards(steer_target_pwm):
            steer_phase = "idle"

def _read_i32_le(reg):
    try:
        data = bus.read_i2c_block_data(I2C_ADDR, reg, 4)
        return int.from_bytes(bytes(data), byteorder="little", signed=True)
    except Exception:
        return None

def read_odometry_ticks():
    left_ticks = _read_i32_le(ENCODER_LEFT_REG)
    right_ticks = _read_i32_le(ENCODER_RIGHT_REG)
    if left_ticks is None or right_ticks is None:
        return None

    # Left encoder has opposite sign convention.
    return (-left_ticks, right_ticks)

def request_steering(steering_offset):
    global desired_steering_offset
    desired_steering_offset = max(-MAX_STEERING_OFFSET, min(MAX_STEERING_OFFSET, int(steering_offset)))

def get_odometry():
    global _last_odom_ticks
    global _last_odom_time

    now = time.monotonic()
    set_robot(int(desired_speed), int(desired_steering_offset), now=now)

    ticks = read_odometry_ticks()
    if ticks is None:
        return 0.0, 0.0, 0.0

    if _last_odom_ticks is None or _last_odom_time is None:
        _last_odom_ticks = ticks
        _last_odom_time = now
        return 0.0, 0.0, 0.0

    dt = now - _last_odom_time
    if dt <= 0.0:
        return 0.0, 0.0, 0.0

    dl = (ticks[0] - _last_odom_ticks[0]) / TICKS_PER_METER
    dr = (ticks[1] - _last_odom_ticks[1]) / TICKS_PER_METER
    v_lin = ((dl + dr) * 0.5) / dt

    _last_odom_ticks = ticks
    _last_odom_time = now
    return dl, dr, v_lin

def read_key_nonblocking():
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return None
    return sys.stdin.read(1)

def run_keyboard_controller():
    global desired_speed

    print("\n" + "="*40)
    print(" ROBOTAXI MASTER CONTROLLER ")
    print("="*40)
    print(" [W] - Forward")
    print(" [S] - Backward")
    print(" [A] - Steer Left")
    print(" [D] - Steer Right")
    print(" [SPACE] - Emergency Stop & Center")
    print(" [Q] - Quit")
    print("="*40)

    current_speed_mps = 0.0
    distance_total_m = 0.0
    distance_net_m = 0.0

    prev_ticks = read_odometry_ticks()
    last_odom_time = time.monotonic()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        next_tick = time.monotonic()
        while True:
            loop_now = time.monotonic()
            k = read_key_nonblocking()
            while k is not None:
                k = k.lower()
                if k == 'w':
                    desired_speed = 30
                    request_steering(0)
                elif k == 's':
                    desired_speed = -30
                    request_steering(0)
                elif k == 'a':
                    request_steering(-MAX_STEERING_OFFSET)
                elif k == 'd':
                    request_steering(MAX_STEERING_OFFSET)
                elif k == ' ':
                    desired_speed = 0
                    request_steering(0)
                elif k == 'q':
                    raise KeyboardInterrupt

                k = read_key_nonblocking()

            set_robot(desired_speed, desired_steering_offset, now=loop_now)

            ticks = read_odometry_ticks()
            if ticks is not None and prev_ticks is not None:
                dt = loop_now - last_odom_time
                if dt > 0:
                    delta_left = ticks[0] - prev_ticks[0]
                    delta_right = ticks[1] - prev_ticks[1]
                    delta_m = ((delta_left + delta_right) * 0.5) / TICKS_PER_METER
                    current_speed_mps = delta_m / dt
                    distance_net_m += delta_m
                    distance_total_m += abs(delta_m)

            if ticks is not None:
                prev_ticks = ticks
                last_odom_time = loop_now

            print(
                f"\rCmd -> V:{desired_speed:+4d} Steer:{desired_steering_offset:+4d} PWM:{current_pwm:4d} | "
                f"Vel:{current_speed_mps:+.3f} m/s | DistNet:{distance_net_m:+.3f} m | DistTot:{distance_total_m:.3f} m   ",
                end="",
                flush=True,
            )

            next_tick += LOOP_PERIOD_S
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            pass

        set_robot(0, 0, now=time.monotonic())
        pi.set_servo_pulsewidth(STEERING_PIN, 0)
        print("\n\nSystem shutdown complete.")

if __name__ == "__main__":
    run_keyboard_controller()