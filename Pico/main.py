from machine import Pin, SoftI2C
import time
import math
from bmp280 import BMP280
from servo import Servo
from bno085 import BNO085

# ==========================================
# ROCKET FLIGHT CONFIGURATION
# ==========================================
SERVO_PIN = 10
SERVO_START_ANGLE = 0       
SERVO_DEPLOY_ANGLE = 90     

# Conversions
METERS_TO_FEET = 3.28084

# ==========================================
# HARDWARE INITIALIZATION
# ==========================================
# Verified stable SoftI2C configuration without external resistors
# BMP280 and BNO085 each get their own bus so a stall/fault on one sensor
# can't hang the other's transactions.
i2c_bmp = SoftI2C(scl=Pin(9, Pin.IN, Pin.PULL_UP), sda=Pin(8, Pin.IN, Pin.PULL_UP), freq=50000)
i2c_imu = SoftI2C(scl=Pin(7, Pin.IN, Pin.PULL_UP), sda=Pin(6, Pin.IN, Pin.PULL_UP), freq=50000)

# Initialize BMP280
bmp = BMP280(i2c_bmp)

# Initialize BNO085 IMU
print("Initializing BNO085 IMU...")
time.sleep_ms(150)
imu = BNO085(i2c_imu)

# Enable required features on the IMU
imu.enable_feature(imu.REPORT_LINEAR_ACCELERATION) # For Inertial Speed
imu.enable_feature(imu.REPORT_ROTATION_VECTOR)      # For Orientation
imu.set_quaternion_euler_vector(imu.REPORT_ROTATION_VECTOR)

# Initialize Deployment Servo and lock it
ejection_servo = Servo(pin_num=SERVO_PIN)
ejection_servo.write_angle(SERVO_START_ANGLE)

print("------------------------------------------")
print("Dual-Source Speed Logging Online (Verified Axis).")
print("Servo locked at {}°. Ready for launch.".format(SERVO_START_ANGLE))
print("------------------------------------------")

deployed = False

# Tracking variables for dual-source speed calculation
last_time = time.ticks_ms() / 1000.0
last_altitude = bmp.altitude * METERS_TO_FEET 

speed_bmp = 0.0  
speed_imu = 0.0  

# Create/overwrite log file with explicit headers
with open("flight_log.txt", "w") as f:
    f.write("Time(s),Altitude(ft),Speed_BMP(ft/s),Speed_IMU(ft/s),Roll(deg),Pitch(deg),Yaw(deg)\n")

while True:
    current_time = time.ticks_ms() / 1000.0
    dt = current_time - last_time
    
    # Always sample altitude first so it's guaranteed to exist for logging/printing
    current_altitude = bmp.altitude * METERS_TO_FEET
    
    if dt > 0:
        # 1. SOURCE A: Calculate Barometric Speed (BMP280)
        speed_bmp = (current_altitude - last_altitude) / dt
        last_altitude = current_altitude
        
        # 2. SOURCE B: Calculate Inertial Speed (BNO085 integration)
        try:
            # acc_linear returns (x, y, z) acceleration in m/s^2 with gravity removed
            ax, ay, az = imu.acc_linear
            
            # FIXED VIA TEST: Your vertical flight axis is Y, and upward acceleration is negative raw
            acc_vertical_fps = -ay * METERS_TO_FEET
            
            # Deadzone filter: Ignore tiny standing sensor noise near zero
            if abs(acc_vertical_fps) > 0.15:
                speed_imu += acc_vertical_fps * dt
            else:
                # Slowly bleed off minor standing drift on the pad
                speed_imu *= 0.95
                
            # FLIGHT PHASE ENFORCEMENT:
            if not deployed:
                # BEFORE APOGEE (Ascent Phase): Speed must be positive
                if speed_imu < 0:
                    speed_imu = 0.0
            else:
                # AFTER APOGEE (Descent Phase): Speed should be negative (downward)
                if speed_imu > 0:
                    speed_imu = 0.0
                
        except Exception:
            pass
            
        last_time = current_time

    # Fetch orientation data safely
    try:
        roll, pitch, yaw = imu.euler
    except Exception:
        roll, pitch, yaw = 0.0, 0.0, 0.0

    # 3. Unified Redundant Logging to Flash Memory
    try:
        with open("flight_log.txt", "a") as f:
            f.write("{:.3f},{:.2f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}\n".format(
                current_time, current_altitude, speed_bmp, speed_imu, roll, pitch, yaw
            ))
    except Exception as e:
        print("Logging Error:", e)

    # 4. Telemetry Printout comparing both sources
    print("Alt: {:.1f}ft | Speed[BMP]: {:5.1f}ft/s | Speed[IMU]: {:5.1f}ft/s | Pitch: {:5.1f}°".format(
        current_altitude, speed_bmp, speed_imu, pitch
    ))

    # 5. Apogee Check & Mechanical Recovery Trigger
    if not deployed and bmp.check_apogee():
        ejection_servo.write_angle(SERVO_DEPLOY_ANGLE)
        deployed = True
        print("\n!!! APOGEE DETECTED - RECOVERY DEPLOYED !!!\n")

    time.sleep(0.05)