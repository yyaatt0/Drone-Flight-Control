from drone_function import *
from PID import PID

'''
Initializing Peripherals Portion 
'''

# IR Sensor Initialization (gauging direction of signal)
front_ir_pin = board.GP27_A1
left_ir_pin = board.GP26_A0
right_ir_pin = board.GP28_A2

front_ir = analogio.AnalogIn(front_ir_pin)
left_ir = analogio.AnalogIn(left_ir_pin)
right_ir = analogio.AnalogIn(right_ir_pin)

# Motor Initialization
top_r_pin = board.GP4 # Clockwise
top_l_pin = board.GP8 # Counter Clockwise
bottom_r_pin = board.GP0 # Counter Clockwise
bottom_l_pin = board.GP12 # Clockwise

# LED Initialization 
blue_led_pin = board.GP18
green_led_pin = board.GP19
yellow_led_pin = board.GP20

blue_led = digitalio.DigitalInOut(blue_led_pin)
green_led = digitalio.DigitalInOut(green_led_pin)
yellow_led = digitalio.DigitalInOut(yellow_led_pin)

blue_led.direction = digitalio.Direction.OUTPUT
green_led.direction = digitalio.Direction.OUTPUT
yellow_led.direction = digitalio.Direction.OUTPUT

# Setting the pins to output PWM signal 
top_r = pwmio.PWMOut(top_r_pin, frequency=50)
top_l = pwmio.PWMOut(top_l_pin, duty_cycle=0, frequency=50)
bottom_r = pwmio.PWMOut(bottom_r_pin, duty_cycle=0, frequency=50)
bottom_l = pwmio.PWMOut(bottom_l_pin, duty_cycle=0, frequency=50)
minimum_pulse = 1000
maximum_pulse = 2000

# Ultrasonic Sensor Initialization; use 'sonar.distance' to read the measurement
sonar = adafruit_hcsr04.HCSR04(trigger_pin=board.GP17, echo_pin=board.GP16)

# Testing ultrasonic sensor upon startup 
try:
    sonar_test = sonar.distance
    del sonar_test
    print("Ultrasonic Sensor connected!")
except RuntimeError:
    print("Ultrasonic Sensor failed.")
    
time.sleep(2)

# Initializing I2C bus
scl_line = board.GP15
sda_line = board.GP14
i2c_initialized = False

while(not i2c_initialized):
    try:
        i2c = busio.I2C(scl_line, sda_line)
        i2c_initialized = True
        pass
    except RuntimeError: # Most likely no pull up resistor attached to pin; but device board has it
        print("I2C initialization failed")
        time.sleep(1)    

expected_devices = 3

# Reading Address from I2C bus 
while not i2c.try_lock():
    pass

i2c_slave = i2c.scan()

while len(i2c_slave) != expected_devices: 
    for device_address in i2c_slave:
        print("Device Address: ", hex(device_address), end="")
        print("\t", end="")
    print("\nRescanning I2C bus...\n")
    i2c_slave = i2c.scan()
    time.sleep(1)

print("All I2C devices connected!")
for device_address in i2c_slave:
    print("Device Address: ", hex(device_address), end="")
    print("\t", end="")
print("\n")
i2c.unlock()
time.sleep(2)

yellow_led.value = True

# VL53L1X Sensor Initialize (Time of Flight)
vl53_sensor = adafruit_vl53l1x.VL53L1X(i2c)
print("Time of Flight Initialized!")
time.sleep(2)

# VL53L1X Sensor Configuration
vl53_sensor.distance_mode = 2   # 1 - SHORT ; 2 - LONG
vl53_sensor.timing_budget = 100

# Begin VL53L1X Sensor; this does not need calibration due to the 
# fact that even when the sensor is not covered, it sets whatever location
# its place on to zero then measures from there
vl53_sensor.start_ranging()

# LSM6DSOX Sensor Initialization (IMU)
lsm_sensor = LSM6DSOX(i2c)
print("IMU Initialized!")
time.sleep(2)

# sensor.accelerometer_data_rate = Rate.RATE_1_66K_HZ
lsm_sensor.accelerometer_data_rate = Rate.RATE_104_HZ  # slower → smoother  
lsm_sensor.gyro_data_rate = Rate.RATE_104_HZ


# I might just calibrate the gyroscope rather than the accelerometer
# because I think the driver already does a great job doing that.
print(("\nCalibrating IMU..."))
x_gyro_bias, y_gyro_bias, z_gyro_bias = calibrate_gyro(lsm_sensor, dt)

green_led.value = True

# Motors need time to boot and configure, so it will spin on a low throttle 
initialize_motors(top_r=top_r, top_l=top_l, bottom_r=bottom_r, bottom_l=bottom_l)

''' 
# PID Controller + Complementary Filter Portion 
'''
# Tuning Variables for Complementary Filter
tau = 0.15

# Hardware Issue: The pitch and roll are swapped, meaning the placement of the IMU is incorrect
# Pitch is actually our roll 
# Roll is actually our pitch 
# Whoever reading this will kill themselves due to the poor and shitty code
filtered_pitch = 0.0
filtered_roll = 0.0 

flat_pitch = 0.0   # This is basically the target pitch that we want when we want the drone to be stationary orientation
flat_roll = 0.0    # This is basically the target pitch that we want when we want the drone to be stationary orientation

print("Setting target pitch and roll...")

for i in range(0, num_of_samples):
    x_accel, y_accel, z_accel = lsm_sensor.acceleration
    x_gyro, y_gyro, z_gyro = lsm_sensor.gyro
    x_gyro -= x_gyro_bias
    y_gyro -= y_gyro_bias
    z_gyro -= z_gyro_bias
    
    
    filtered_pitch, filtered_roll = get_filtered_pitch_roll(filtered_pitch, filtered_roll, x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro, dt)
    flat_pitch += filtered_pitch
    flat_roll += filtered_roll
    time.sleep(dt)
    
# Now we have our target pitch and roll for the PID whenever we want to have it stable
flat_pitch /= num_of_samples
flat_roll /= num_of_samples

print(f"Stable Pitch: {flat_pitch}, Stable Roll: {flat_roll}")
target_altitude = 15.0

# Tuning variables for each PID controller
kp_pitch = 50
ki_pitch = 25
kd_pitch = 0

kp_roll = 50
ki_roll = 25
kd_roll = 0

kp_thrust = 30
ki_thrust = 15
kd_thrust = 0

# Initializing PID objects
pitch_pid = PID(kp=kp_pitch, ki=ki_pitch, kd=kd_pitch)
roll_pid  = PID(kp=kp_roll, ki=ki_roll, kd=kd_roll)
thrust_pid = PID(kp=kp_thrust, ki=ki_thrust, kd=kd_thrust)

# base_throttle_l = 1800
# base_throttle_r = 1800 
throttle = 1050
timeout_time = 5 # 10 seconds
# Colors of the LED To indicate that the drone will start flying
blue_led.value = False
yellow_led.value = False
blue_led.value = False

time.sleep(2)

''' This loop demonstrates the IR sensor controlling the drone '''
# while True:
#     while(adc_voltage(front_ir.value) < 2.0):
#         set_all_motor(top_r, top_l, bottom_r, bottom_l, throttle, throttle, throttle, throttle)
#         time.sleep(dt)
#     set_all_motor(top_r, top_l, bottom_r, bottom_l, 0, 0, 0, 0)
#     time.sleep(dt)
    
    
for throttle in range(1000, 1800, 50):
    set_all_motor(top_r, top_l, bottom_r, bottom_l, throttle, throttle, throttle, throttle)
    time.sleep(1)

start_time = time.time()

''' Initial loop to get the drone into the air and stable'''
while (time.time() - start_time < timeout_time):
    height = vl53_sensor.distance
    x_accel, y_accel, z_accel = lsm_sensor.acceleration
    x_gyro, y_gyro, z_gyro = lsm_sensor.gyro
    x_gyro -= x_gyro_bias
    y_gyro -= y_gyro_bias
    z_gyro -= z_gyro_bias
    
    # Getting clean data and smoother data
    filtered_pitch, filtered_roll = get_filtered_pitch_roll(filtered_pitch, filtered_roll, x_accel, y_accel, z_accel, x_gyro, y_gyro, tau, dt)
    
    # Getting updated values to update the error
    pitch_output = pitch_pid.update(flat_pitch, filtered_pitch, dt)
    roll_output = roll_pid.update(flat_roll, filtered_pitch, dt)
    thrust_output = thrust_pid.update(target_altitude, height, dt)
    
    ''' Throttle Altitude PID '''
    # Calculates the throttle to update
    top_r_throttle = base_throttle_r + thrust_output + pitch_output + roll_output 
    top_l_throttle = base_throttle_l + thrust_output + pitch_output - roll_output
    bottom_r_throttle = base_throttle_r + thrust_output - pitch_output + roll_output
    bottom_l_throttle = base_throttle_l + thrust_output - pitch_output - roll_output
    
    # Sets the throttle to the motors
    set_all_motor(top_r, top_l, bottom_r, bottom_l, top_r_throttle, top_l_throttle, bottom_r_throttle, bottom_l_throttle)
    
    # For personal reference if the motors are off
    # print(f"Filtered Pitch: {filtered_pitch}, Filtered Roll: {filtered_roll}, Altitude: {height}, IR Value: {adc_voltage(front_ir.value)}")
    print(f"top_r: {top_r_throttle}, top_l: {top_l_throttle}, botton_r: {bottom_r_throttle}, bottom_l: {bottom_l_throttle}, Altitude: {height}, Error: {thrust_pid.previous_error}")
    time.sleep(dt)
    
''' Middle loop for the companion logic part '''
# yellow_led.value = False
# green_led.value = False
# blue_led.value = True
# start_time = time.time()
# ir_time = None # This time will reset if any of the IR signals was read high during a loop iteration

# while(ir_time - start_time < 15): # So if no IR signal was read for more than 15 seconds, the loop will end and start decending 
#     height = vl53_sensor.distance
#     x_accel, y_accel, z_accel = lsm_sensor.acceleration
#     x_gyro, y_gyro, z_gyro = lsm_sensor.gyro
#     x_gyro -= x_gyro_bias
#     y_gyro -= y_gyro_bias
#     z_gyro -= z_gyro_bias
    
#     # Final cleaned data to be used for later application about the orientation of the drone
#     filtered_pitch, filtered_roll = get_filtered_pitch_roll(filtered_pitch, filtered_roll, x_accel, y_accel, z_accel, x_gyro, y_gyro, tau, dt)
    
#     # Getting an output to correct error 
#     pitch_output = pitch_pid.update(flat_pitch, filtered_pitch, dt)
#     roll_output = roll_pid.update(flat_roll, filtered_pitch, dt)
#     thrust_output = thrust_pid.update(target_altitude, height, dt)
    
#     # Updates the throttle
#     top_r_throttle = base_throttle + thrust_output + pitch_output + roll_output 
#     top_l_throttle = base_throttle + thrust_output + pitch_output - roll_output
#     bottom_r_throttle = base_throttle + thrust_output - pitch_output + roll_output
#     bottom_l_throttle = base_throttle + thrust_output - pitch_output - roll_output
    
#     ''' 
#         So we want the PID controller for the altitude when we want it to get some position in the air. If we attempt to move 
#         in the x-axis and y-axis, the time of flight sensor will read different readings. Then if the altitude PID controller 
#         is enabled, it will attemp to adjust the throttle, which we do not want.
        
#         So if we only disable for a short time if we want x y movement and reenable back after it is done. 
#     '''
    
#     set_all_motor(top_r, top_l, bottom_r, bottom_l, top_r_throttle, top_l_throttle, bottom_r_throttle, bottom_l_throttle)
    
#     print(f"Filtered Pitch: {filtered_pitch}, Filtered Roll: {filtered_roll}, Altitude: {height}, IR Value: {adc_voltage(front_ir.value)}")
#     time.sleep(dt)

''' Final loop to decend the drone safely '''
# target_altitude = 0.0
# yellow_led.value = True
# green_led.value = False
# blue_led.value = False
# start_time = time.time()

# while (time.time() - start_time < 10):
#     height = vl53_sensor.distance
#     x_accel, y_accel, z_accel = lsm_sensor.acceleration
#     x_gyro, y_gyro, z_gyro = lsm_sensor.gyro
#     x_gyro -= x_gyro_bias
#     y_gyro -= y_gyro_bias
#     z_gyro -= z_gyro_bias
    
#     # Getting clean data and smoother data
#     filtered_pitch, filtered_roll = get_filtered_pitch_roll(filtered_pitch, filtered_roll, x_accel, y_accel, z_accel, x_gyro, y_gyro, tau, dt)
    
#     # Getting updated values to update the error
#     pitch_output = pitch_pid.update(flat_pitch, filtered_pitch, dt)
#     roll_output = roll_pid.update(flat_roll, filtered_pitch, dt)
#     thrust_output = thrust_pid.update(target_altitude, height, dt)
    
#     ''' Throttle Altitude PID '''
#     # Calculates the throttle to update
#     top_r_throttle = base_throttle + thrust_output + pitch_output + roll_output 
#     top_l_throttle = base_throttle + thrust_output + pitch_output - roll_output
#     bottom_r_throttle = base_throttle + thrust_output - pitch_output + roll_output
#     bottom_l_throttle = base_throttle + thrust_output - pitch_output - roll_output
    
#     # Sets the throttle to the motors
#     set_all_motor(top_r, top_l, bottom_r, bottom_l, top_r_throttle, top_l_throttle, bottom_r_throttle, bottom_l_throttle)
    
#     # Hopefully getting stable reading
#     print(f"Filtered Pitch: {filtered_pitch}, Filtered Roll: {filtered_roll}, Altitude: {height}, IR Value: {adc_voltage(front_ir.value)}")
#     time.sleep(dt)
    
# Ensuring the motors' throttle is actually set to zero after given like 10 seconds to decend 
# set_all_motor(top_r, top_l, bottom_r, bottom_l, 1000, 1000, 1000, 1000)