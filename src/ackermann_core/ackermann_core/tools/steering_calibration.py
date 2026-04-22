import pigpio
import sys
import tty
import termios
import time

STEERING_PIN = 12

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

pi = pigpio.pi()
if not pi.connected:
    print("ERROR: pigpiod daemon not running. Execute 'sudo pigpiod'.")
    sys.exit()

# --- INITIAL CALIBRATION VALUES ---
center_pwm = 1475  
left_kick = 0    
right_kick = -50  
current_pwm = center_pwm 

def move_smooth(target_pwm, apply_kick=False, kick_val=0):
    global current_pwm
    
    # Calculate primary target (includes overshoot for backlash compensation if applied)
    primary_target = (center_pwm + kick_val) if apply_kick else target_pwm
        
    # Slew-rate limited movement to primary target
    if primary_target != current_pwm:
        step = 15 if primary_target > current_pwm else -15
        while abs(primary_target - current_pwm) > abs(step):
            current_pwm += step
            pi.set_servo_pulsewidth(STEERING_PIN, current_pwm)
            time.sleep(0.01)
            
        current_pwm = primary_target
        pi.set_servo_pulsewidth(STEERING_PIN, current_pwm)
        
    # Settle and return to true center if backlash compensation was applied
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

def print_menu():
    print("\n" + "="*45)
    print(" 🛠️  STEERING CALIBRATION TOOL (SMOOTH) 🛠️")
    print("="*45)
    print(f" [O] / [P] -> Modify Center: {center_pwm}")
    print(f" [Z] / [X] -> Modify Left Kick: {left_kick}")
    print(f" [C] / [V] -> Modify Right Kick: {right_kick}")
    print("-" * 45)
    print(" [1] -> TEST: Steer Left & Return to Center")
    print(" [2] -> TEST: Steer Right & Return to Center")
    print(" [Q] -> Quit & Save Output")
    print("="*45)
    print("Press a key...", end="", flush=True)

try:
    # Gentle initialization to center
    pi.set_servo_pulsewidth(STEERING_PIN, center_pwm)
    time.sleep(0.5)
    
    while True:
        print_menu()
        cmd = getch().lower()
        
        if cmd == 'o': center_pwm -= 5
        elif cmd == 'p': center_pwm += 5
        elif cmd == 'z': left_kick -= 5
        elif cmd == 'x': left_kick += 5
        elif cmd == 'c': right_kick -= 5
        elif cmd == 'v': right_kick += 5
        
        elif cmd == '1':
            print(f"\n>> Steer Left (1200)...", end="", flush=True)
            move_smooth(1200)
            time.sleep(1) 
            print(f" Return to Center ({center_pwm}) with Left Kick ({left_kick})...")
            move_smooth(center_pwm, apply_kick=True, kick_val=left_kick)
            
        elif cmd == '2':
            print(f"\n>> Steer Right (1800)...", end="", flush=True)
            move_smooth(1800)
            time.sleep(1)
            print(f" Return to Center ({center_pwm}) with Right Kick ({right_kick})...")
            move_smooth(center_pwm, apply_kick=True, kick_val=right_kick)
            
        elif cmd == 'q':
            break

finally:
    print("\n\n✅ CALIBRATION COMPLETE. Update your HAL with these values:")
    print(f"CENTER_PWM = {center_pwm}")
    print(f"LEFT_KICK = {left_kick}")
    print(f"RIGHT_KICK = {right_kick}")
    pi.stop()
