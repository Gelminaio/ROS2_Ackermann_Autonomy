import smbus2
import pigpio
import time
import sys
import tty
import termios

# Hardware Configuration
I2C_ADDR = 0x34
MOTOR_REG = 51
STEERING_PIN = 12

# Calibration Parameters
CENTER_PWM = 1475
LEFT_KICK = 30
RIGHT_KICK = -100

current_pwm = CENTER_PWM

pi = pigpio.pi()
bus = smbus2.SMBus(1)

if not pi.connected:
    print("ERROR: pigpiod daemon not running. Execute 'sudo pigpiod' first.")
    sys.exit()

# Initialize steering position
pi.set_servo_pulsewidth(STEERING_PIN, CENTER_PWM)
time.sleep(0.5)

def set_robot(speed, steering_offset):
    global current_pwm
    
    # 1. Drive Control (I2C)
    def to_byte(v): return v if v >= 0 else (256 + v)
    
    vel_left = -speed
    vel_right = speed
    packet = [to_byte(vel_left), to_byte(vel_left), to_byte(vel_right), to_byte(vel_right)]
    
    try:
        bus.write_i2c_block_data(I2C_ADDR, MOTOR_REG, packet)
    except Exception:
        pass 

    # 2. Steering Control (PWM)
    target_pwm = CENTER_PWM + steering_offset
    target_pwm = max(1000, min(2000, target_pwm)) 
    
    apply_kick = False
    kick_val = 0
    
    # Backlash compensation check
    if target_pwm == CENTER_PWM and abs(current_pwm - CENTER_PWM) > 50:
        apply_kick = True
        kick_val = LEFT_KICK if current_pwm < CENTER_PWM else RIGHT_KICK

    primary_target = (CENTER_PWM + kick_val) if apply_kick else target_pwm
        
    # Slew rate limiter (Primary movement)
    if primary_target != current_pwm:
        step = 15 if primary_target > current_pwm else -15
        while abs(primary_target - current_pwm) > abs(step):
            current_pwm += step
            pi.set_servo_pulsewidth(STEERING_PIN, current_pwm)
            time.sleep(0.01)
            
        current_pwm = primary_target
        pi.set_servo_pulsewidth(STEERING_PIN, current_pwm)
        
    # Slew rate limiter (Post-kick centering)
    if apply_kick:
        time.sleep(0.08) 
        if target_pwm != current_pwm:
            step = 5 if target_pwm > current_pwm else -5
            while abs(target_pwm - current_pwm) > abs(step):
                current_pwm += step
                pi.set_servo_pulsewidth(STEERING_PIN, current_pwm)
                time.sleep(0.01)
                
            current_pwm = target_pwm
            pi.set_servo_pulsewidth(STEERING_PIN, current_pwm)

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

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

try:
    speed = 0
    steering = 0
    while True:
        k = getch().lower()
        if k == 'w':
            speed = 30
            steering = 0
        elif k == 's':
            speed = -30
            steering = 0
        elif k == 'a':
            steering = -300
        elif k == 'd':
            steering = 300
        elif k == ' ':
            speed = 0
            steering = 0
        elif k == 'q':
            break
        
        set_robot(speed, steering)
        print(f"\rStatus -> Speed: {speed} | Target PWM: {CENTER_PWM + steering}    ", end="")

except KeyboardInterrupt:
    pass
finally:
    set_robot(0, 0)
    pi.set_servo_pulsewidth(STEERING_PIN, 0)
    print("\n\nSystem shutdown complete.")
