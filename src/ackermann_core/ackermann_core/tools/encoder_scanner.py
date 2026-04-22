import smbus2
import time
import sys

I2C_ADDR = 0x34
bus = smbus2.SMBus(1)


REG_ENCODER_LEFT = 60  
REG_ENCODER_RIGHT = 68 

def read_encoder(start_reg):
    try:
        
        data = bus.read_i2c_block_data(I2C_ADDR, start_reg, 4)
        
        ticks = int.from_bytes(data, byteorder='little', signed=True)
        return ticks
    except Exception:
        return None

print("\n" + "="*45)
print(" 🔄 ENCODER ODOMETRY TOOL 🔄")
print("="*45)
print("Reading 32-bit tick counts from I2C bus...")
print("Press Ctrl+C to exit.\n")

try:
    while True:
        ticks_l = read_encoder(REG_ENCODER_LEFT)
        if ticks_l is not None:
            ticks_l = -ticks_l 
            
        ticks_r = read_encoder(REG_ENCODER_RIGHT)
        
        if ticks_l is not None and ticks_r is not None:
            sys.stdout.write(f"\rLeft Wheel Ticks: {ticks_l:8d} | Right Wheel Ticks: {ticks_r:8d}")
            sys.stdout.flush()
            
        time.sleep(0.05) 
        
except KeyboardInterrupt:
    print("\n\nTest finishedi.")
