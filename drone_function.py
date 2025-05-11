# import numpy as np
import time
import math
import board
import digitalio
import analogio
import busio
import pwmio
import adafruit_hcsr04
import displayio
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lsm6ds import Rate
import adafruit_vl53l1x

dt = 0.01 # Change in time; this will be very small or something

# The number of initial sample readings for the IMU to read and calibrate based off of that
num_of_samples = 100

# This throttle is for moving forwards, backwards, left and right 
adjustment_throttle = 300
yaw_throttle = 1100 # Some low throttle to turn the direction of the motor

# Converting Raw ADC value to voltage 
# Gon be using this for IR sensor
def adc_voltage(initValue) -> float:  
    return (initValue * 3.3) / 65535

def set_throttle(us):
    # us = microsecond pulse (1000 = min, 2000 = max)
    return int(us * 65535 / 20000)  # 20 ms period = 50 Hz

# Function will return a Gyroscope Bias that will be added on top of the measurement 
# So this function will return three biases: X, Y, Z for the gyroscope measurement
def calibrate_gyro(init_IMU: LSM6DSOX, init_dt: float) -> float:
    x_gyro_sum = 0.0
    y_gyro_sum = 0.0
    z_gyro_sum = 0.0
    
    for i in range(0, num_of_samples):
        x, y, z = init_IMU.gyro
        x_gyro_sum += x
        y_gyro_sum += y
        z_gyro_sum += z
        time.sleep(init_dt)
    
    x_gyro_bias = x_gyro_sum / num_of_samples
    y_gyro_bias = y_gyro_sum / num_of_samples
    z_gyro_bias = z_gyro_sum / num_of_samples
    
    return x_gyro_bias, y_gyro_bias, z_gyro_bias

# Function will return a Accelerometer Bias that will be added on top of the measurement
# So this function will return three biases: X, Y, Z for the accelerometer measurement 
# This is not actual correct calibration of the IMU but will setting things to zero
def calibrate_accel(init_IMU: LSM6DSOX, init_dt: float) -> float:    
    x_accel_sum = 0.0
    y_accel_sum = 0.0
    z_accel_sum = 0.0

    for i in range(0, num_of_samples):
        x, y, z = init_IMU.acceleration
        x_accel_sum += x
        y_accel_sum += y
        z_accel_sum += z
        time.sleep(init_dt)

    x_accel_bias = x_accel_sum / num_of_samples
    y_accel_bias = y_accel_sum / num_of_samples
    z_accel_bias = z_accel_sum / num_of_samples
    
    return x_accel_bias, y_accel_bias, z_accel_bias
    

# Function to convert input value with their range and convert to the appropriate value of another range 
def map_value(initValue, in_min, in_max, out_min, out_max) -> float: 
    range_in = in_max - in_min
    range_out = out_max - out_min
    
    output = (((initValue - in_min) / range_in) * range_out) + out_min
    
    if(out_min > out_max):
        if(output > out_min):
            output = out_min
        else:
            output = out_max
    elif (output < out_min):
        output = out_min
    else:
        output = out_max
        
    return output

# Calculates the pitch and roll with combination of the gyroscope. 
# Must call this function inside the 'while True' loop to update the filtered pitch and roll at every iteration 
# Most likely we NOT using this bih 
def get_filtered_pitch_roll(init_pitch, init_roll, ax, ay, az, gx, gy, tau, dt) -> float:
    alpha = tau / (tau + dt)
    
    pitch = math.atan2(ax, math.sqrt(ay ** 2 + az ** 2))
    roll = math.atan2(ay, math.sqrt(ax ** 2 + az ** 2))
    
    '''
    Applying Complementary Filter
    High Pass Filter is applied on the Gyroscope for its accurate short term measurement
    Low Pass Filter is applied on the Accelerometer for its accurate long term measurement
    '''
    filtered_pitch = alpha * (init_pitch + gy * dt) + (1 - alpha) * pitch
    filtered_roll = alpha * (init_roll + gx * dt) + (1 - alpha) * roll 
    
    return filtered_pitch, filtered_roll 

def initialize_motors(top_r, top_l, bottom_r, bottom_l) -> None:
    # Setting the throttle to 0%
    print("Initialize all motors...")
    top_r.duty_cycle = set_throttle(1000)
    top_l.duty_cycle = set_throttle(1000)
    bottom_r.duty_cycle = set_throttle(1000)
    bottom_l.duty_cycle = set_throttle(1000)
    
    # Time to power and configure; do for 10s
    for i in range(0, 10, 1):
        print("#", end="")
        time.sleep(1) 
    
    print("\nStarting motors...")
    
    print("Increasing throttle...")
    for us in range(1000, 1125, 25):  # ramp to 1500 µs
        print(f"Throttle {((us - 1000) / 1000.0) * 100}%")
        top_r.duty_cycle = set_throttle(us)
        top_l.duty_cycle = set_throttle(us)
        bottom_r.duty_cycle = set_throttle(us)
        bottom_l.duty_cycle = set_throttle(us)
        time.sleep(1)
        
    # for us in range(0, 10, 1):  # ramp to 1500 µs
    #     top_r.duty_cycle = set_throttle(1100)
    #     top_l.duty_cycle = set_throttle(1100)
    #     bottom_r.duty_cycle = set_throttle(1100)
    #     bottom_l.duty_cycle = set_throttle(1100)
    #     time.sleep(1)
    
    print("Decreasing throttle...")
    for us in range(1100, 975, -25):  # ramp to 1500 µs
        print(f"Throttle {((us - 1000) / 1000.0) * 100}%")
        top_r.duty_cycle = set_throttle(us)
        top_l.duty_cycle = set_throttle(us)
        bottom_r.duty_cycle = set_throttle(us)
        bottom_l.duty_cycle = set_throttle(us)
        time.sleep(1)
                
    # Settting the throttle to 0%
    print("\nStopping motors...")
    top_r.duty_cycle = set_throttle(1000)
    top_l.duty_cycle = set_throttle(1000)
    bottom_r.duty_cycle = set_throttle(1000)
    bottom_l.duty_cycle = set_throttle(1000)
    time.sleep(3)

# For the movement, consider a smoother transition instead of immediately changing the throttle, test to see how the drone performs

# For now we want all the motors to have the same throttle
# We may want to call PID to keep it stationary 
def set_all_motor(top_r_motor, top_l_motor, bottom_r_motor, bottom_l_motor, top_r_throttle, top_l_throttle, bottom_r_throttle, bottom_l_throttle):
    top_r_throttle = max(min(top_r_throttle, 2000), 1000)
    top_l_throttle = max(min(top_l_throttle, 2000), 1000)
    bottom_r_throttle = max(min(bottom_r_throttle, 2000), 1000)
    bottom_l_throttle = max(min(bottom_l_throttle, 2000), 1000)
    
    top_r_motor.duty_cycle = set_throttle(top_r_throttle)
    top_l_motor.duty_cycle = set_throttle(top_l_throttle)
    bottom_r_motor.duty_cycle = set_throttle(bottom_r_throttle)
    bottom_l_motor.duty_cycle = set_throttle(bottom_l_throttle)
