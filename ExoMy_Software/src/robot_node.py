#!/usr/bin/env python
import time
from exomy.msg import RoverCommand, MotorCommands, Screen
import rospy
from rover import Rover
import message_filters
from std_msgs.msg import String
import i2c

# Global variables.
# ReneB: I2C address of Atmega328P.
slaveAddressAtmega328P = 0x04
global exomy
exomy = Rover()


def joy_callback(message):
    cmds = MotorCommands()

    if message.motors_enabled is True:
        exomy.setLocomotionMode(message.locomotion_mode)

        cmds.motor_angles = exomy.joystickToSteeringAngle(
            message.vel, message.steering)
        cmds.motor_speeds = exomy.joystickToVelocity(
            message.vel, message.steering)
    else:
        cmds.motor_angles = exomy.joystickToSteeringAngle(0, 0)
        cmds.motor_speeds = exomy.joystickToVelocity(0, 0)

    robot_pub.publish(cmds)

if __name__ == '__main__':
    rospy.init_node('robot_node')
    rospy.loginfo("Starting the robot node")
    global robot_pub
    joy_sub = rospy.Subscriber(
        "/rover_command", RoverCommand, joy_callback, queue_size=1)

    rate = rospy.Rate(10)

    robot_pub = rospy.Publisher("/motor_commands", MotorCommands, queue_size=1)
    # ReneB: Added publishers for battery voltage and solarpanel voltage.
    battery_voltage_pub = rospy.Publisher('/battery_voltage', String, queue_size=1)
    solarpanel_voltage_pub = rospy.Publisher('/solarpanel_voltage', String, queue_size=1)

    # ReneB: Create while loop with sleep to publish battery voltage and solarpanel voltage every second.
    while not rospy.is_shutdown():
        # Read battery voltage from Atmega328P
        i2c.write_byte(slaveAddressAtmega328P, 0, 100)  # Command to indicate a read is going to follow.
        i2c.write_byte(slaveAddressAtmega328P, 0, 128)  # 129 means read battery voltage.
        rospy.sleep(0.1) # Give Atmega328P time to process.
        battery_voltage = i2c.read_byte(slaveAddressAtmega328P, 0)
        battery_voltage_pub.publish(str(battery_voltage))
        i2c.write_byte(slaveAddressAtmega328P, 0, 100)  # Command to indicate a read is going to follow.
        i2c.write_byte(slaveAddressAtmega328P, 0, 129)  # 129 means read battery voltage.
        rospy.sleep(0.1) # Give Atmega328P time to process.
        solarpanel_voltage = i2c.read_byte(slaveAddressAtmega328P, 0)
        solarpanel_voltage_pub.publish(str(solarpanel_voltage))
        rospy.sleep(1)

    #rospy.spin()  # ReneB: Commented out because of while loop with sleep above.
