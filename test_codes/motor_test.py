import pigpio
import time

# GPIO pins
PWM_PIN = 18   # Must support hardware PWM
DIR_PIN = 23

# Setup pigpio
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio daemon not running")

# Set modes
pi.set_mode(PWM_PIN, pigpio.OUTPUT)
pi.set_mode(DIR_PIN, pigpio.OUTPUT)

# PWM settings
PWM_FREQUENCY = 20000      # 20 kHz (safe for DC motors)
DUTY_CYCLE = 150000        # Range: 0–1,000,000 (~15% speed)

def motor_stop():
    pi.hardware_PWM(PWM_PIN, PWM_FREQUENCY, 0)

def motor_forward():
    pi.write(DIR_PIN, 1)
    pi.hardware_PWM(PWM_PIN, PWM_FREQUENCY, DUTY_CYCLE)

def motor_backward():
    pi.write(DIR_PIN, 0)
    pi.hardware_PWM(PWM_PIN, PWM_FREQUENCY, DUTY_CYCLE)

try:
    # Forward for 3 seconds
    motor_forward()
    time.sleep(3)
    print("forward")

    # Stop before direction change
    motor_stop()
    time.sleep(0.5)

    # Backward for 3 seconds
    motor_backward()
    time.sleep(3)

    print("backward")
    # Final stop
    motor_stop()

finally:
    pi.hardware_PWM(PWM_PIN, PWM_FREQUENCY, 0)
    pi.stop() 