#!/usr/bin/env python

import time
import Adafruit_PCA9685
from . import i2c
# The Adafruit PCA9685 Servo HAT uses the Raspberry Pi default I2C bus (I2C-1) on pin 3 (SDA) and pin 5 (SCL).
# To prevent simultanuous access from another thread (ROS node) the lock 'i2c.i2c_lock' is used.
# There is also another I2C bus (I2C-0) on pin 27 (SDA) and pin 28 (SCL) but this is provided by the video core (not the ARM core), does not have pull-up resisors and is used by the camera.

class Motors():
    '''
    Motors class contains all functions to control the steering and driving motors.
    '''

    # Define wheel names
    FL, FR, CL, CR, RL, RR = range(0, 6)

    # Motor commands are assuming positiv=driving_forward, negative=driving_backwards.
    # The driving direction of the left side has to be inverted for this to apply to all wheels.
    wheel_directions = [-1, 1, -1, 1, -1, 1]

    # 1 fl-||-fr 2
    #      ||
    # 3 cl-||-cr 4
    # 5 rl====rr 6

    def __init__(self, parameters):

        # Dictionary containing the pins of all motors
        self.pins = {
            'drive': {},
            'steer': {}
        }

        # Set variables for the GPIO motor pins
        self.pins['drive'][self.FL] = parameters['pin_drive_fl']
        self.pins['steer'][self.FL]  = parameters['pin_steer_fl']

        self.pins['drive'][self.FR] = parameters['pin_drive_fr']
        self.pins['steer'][self.FR]  = parameters['pin_steer_fr']

        self.pins['drive'][self.CL] = parameters['pin_drive_cl']
        self.pins['steer'][self.CL]  = parameters['pin_steer_cl']

        self.pins['drive'][self.CR] = parameters['pin_drive_cr']
        self.pins['steer'][self.CR] = parameters['pin_steer_cr']

        self.pins['drive'][self.RL] = parameters['pin_drive_rl']
        self.pins['steer'][self.RL] = parameters['pin_steer_rl']

        self.pins['drive'][self.RR] = parameters['pin_drive_rr']
        self.pins['steer'][self.RR] = parameters['pin_steer_rr']

        # PWM characteristics
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)  # Hz

        self.steering_pwm_neutral = [None] * 6

        self.steering_pwm_neutral[self.FL] = parameters['steer_pwm_neutral_fl']
        self.steering_pwm_neutral[self.FR] = parameters['steer_pwm_neutral_fr']
        self.steering_pwm_neutral[self.CL] = parameters['steer_pwm_neutral_cl']
        self.steering_pwm_neutral[self.CR] = parameters['steer_pwm_neutral_cr']
        self.steering_pwm_neutral[self.RL] = parameters['steer_pwm_neutral_rl']
        self.steering_pwm_neutral[self.RR] = parameters['steer_pwm_neutral_rr']
        self.steering_pwm_range = parameters['steer_pwm_range']

        self.driving_pwm_neutral = parameters['drive_pwm_neutral'] 
        self.driving_pwm_range = parameters['drive_pwm_range']

        # Set the GPIO to software PWM at 'Frequency' Hertz
        self.driving_motors = [None] * 6
        self.steering_motors = [None] * 6

        # Set steering motors to neutral values (straight)
        for wheel_name, motor_pin in self.pins['steer'].items():
            with i2c.i2c_lock:
                self.pwm.set_pwm(motor_pin, 0,
                             self.steering_pwm_neutral[wheel_name])
            time.sleep(0.1)

        self.wiggle()

    def wiggle(self):
        # Wiggle the two front wheels on startup to cnfirm functionality
        time.sleep(0.1)
        with i2c.i2c_lock:
            self.pwm.set_pwm(self.pins['steer'][self.FL], 0,
                         int(self.steering_pwm_neutral[self.FL] + self.steering_pwm_range * 0.3))
        time.sleep(0.1)
        with i2c.i2c_lock:
            self.pwm.set_pwm(self.pins['steer'][self.FR], 0,
                         int(self.steering_pwm_neutral[self.FR] + self.steering_pwm_range * 0.3))
        time.sleep(0.3)
        with i2c.i2c_lock:
            self.pwm.set_pwm(self.pins['steer'][self.FL], 0,
                         int(self.steering_pwm_neutral[self.FL] - self.steering_pwm_range * 0.3))
        time.sleep(0.1)
        with i2c.i2c_lock:
            self.pwm.set_pwm(self.pins['steer'][self.FR], 0,
                         int(self.steering_pwm_neutral[self.FR] - self.steering_pwm_range * 0.3))
        time.sleep(0.3)
        with i2c.i2c_lock:
            self.pwm.set_pwm(self.pins['steer'][self.FL], 0,
                         int(self.steering_pwm_neutral[self.FL]))
        time.sleep(0.1)
        with i2c.i2c_lock:
            self.pwm.set_pwm(self.pins['steer'][self.FR], 0,
                         int(self.steering_pwm_neutral[self.FR]))
        time.sleep(0.3)

    def setSteering(self, steering_command):
        # Loop through pin dictionary. The items key is the wheel_name and the value the pin.
        for wheel_name, motor_pin in self.pins['steer'].items():
            duty_cycle = int(
                self.steering_pwm_neutral[wheel_name] + steering_command[wheel_name]/90.0 * self.steering_pwm_range)
            with i2c.i2c_lock:
                self.pwm.set_pwm(motor_pin, 0, duty_cycle)

    def setDriving(self, driving_command):
        # Loop through pin dictionary. The items key is the wheel_name and the value the pin.
        for wheel_name, motor_pin in self.pins['drive'].items():
            duty_cycle = int(self.driving_pwm_neutral +
                             driving_command[wheel_name]/100.0 * self.driving_pwm_range * self.wheel_directions[wheel_name])
            # ReneB: completely switch off the drive motors when duty cycle is close to neutral position, see https://learn.adafruit.com/16-channel-pwm-servo-driver
            # This to eleminate the need to calibrate the drive motors to zero speed using the potentiometer every time.
            if abs(duty_cycle - self.driving_pwm_neutral) < 10:
                duty_cycle = 4096
            with i2c.i2c_lock:
                self.pwm.set_pwm(motor_pin, 0, duty_cycle)

    def stopMotors(self):
        # Set driving wheels to neutral position to stop them
        duty_cycle = int(self.driving_pwm_neutral)

        for wheel_name, motor_pin in self.pins['drive'].items():
            #self.pwm.set_pwm(motor_pin, 0, duty_cycle)
            # ReneB: completely switch off the drive motor when duty cycle is close to neutral position, see https://learn.adafruit.com/16-channel-pwm-servo-driver
            # This to eleminate the need to calibrate the drive motors to zero speed using the potentiometer every time.
            # stopMotors is called after a Watchdog timeout which is after 5 seconds of inactivity.
            with i2c.i2c_lock:
                self.pwm.set_pwm(motor_pin, 0, 4096)
