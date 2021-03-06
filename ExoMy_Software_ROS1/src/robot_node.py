#!/usr/bin/env python
import time
from exomy.msg import RoverCommand, MotorCommands, Screen
import rospy
from rover import Rover
import message_filters
from std_msgs.msg import String
import i2c
from subprocess import call
import own_util

# Global variables.
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

# ReneB: callback to handle all own button events from the webpage.
def own_button_callback(message):
    if message.data == "lights_on":
      i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 1)  # Turn lights on
    elif message.data == "lights_off":
      i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 2)  # Turn lights off
    elif message.data == "goto_sleep":
        i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 42)  # Send sleep command
        # After sending the sleep command to the Atmega328P, the Atmega328P will send back '42' as hardware status indicating it will put the MyExoMy to sleep after a delay.
        # This hardware status is read below so the Raspberry Pi can properly shut down.


if __name__ == '__main__':
    rospy.init_node('robot_node')
    rospy.loginfo("Starting the robot node")
    global robot_pub
    joy_sub = rospy.Subscriber(
        "/rover_command", RoverCommand, joy_callback, queue_size=1)
    # ReneB: subscribe to own_button topic, meant for buttons on the web page to turn the lights on and go to sleep.
    joy_sub = rospy.Subscriber(
        "/own_button", String, own_button_callback, queue_size=1)

    rate = rospy.Rate(10)

    robot_pub = rospy.Publisher("/motor_commands", MotorCommands, queue_size=1)
    # ReneB: Added publishers for status.
    battery_status_pub = rospy.Publisher('/battery_status', String, queue_size=1)
    solarpanel_status_pub = rospy.Publisher('/solarpanel_status', String, queue_size=1)
    wifi_status_pub = rospy.Publisher('/wifi_status', String, queue_size=1)
    sleep_status_pub = rospy.Publisher('/sleep_status', String, queue_size=1)

    # ReneB: The node.js WebRTC signal server and Chromium running on the Raspberry Pi host ideally would be started from here.
    # However, this is quite difficult from the Docker container through SSH.
    # So now these are stopped and started on the Raspberry Pi host from /home/pi/ExoMy_Software/start_webrtc which is called from /etc/xdg/lxsession/LXDE-pi/autostart.
    # own_util.HostStopVideoStream()
    # own_util.HostStartVideoStream()
    
    # ReneB: Create while loop with sleep to publish the status every second.
    while not rospy.is_shutdown():
        # Read hardware status from Atmega328P
        i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 100)  # Command to indicate a read is going to follow.
        i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 255)  # 255 means read hardware status.
        rospy.sleep(i2c.i2cDelay) # Give Atmega328P time to process.
        hardware_status = i2c.read_byte(i2c.slaveAddressAtmega328P, 0)
        # A hardware status of 42 means that the Atmega328P received a request to put the MyExoMy to sleep after a delay.
        # This sleep request can come from the Web Interface (RaspBerry Pi) and also from the Atmega328P itself when there is no motion (activity) detected for a certain period of time.
        if hardware_status == 42:
          # ReneB: If we get here it means that the ATmega328P is going to put the MyExoMy to sleep after a short waiting period.
          # After this short period the ATmega328P will switch off the power to the Raspberry Pi. During this period we shutdown the Raspberry Pi in a proper way.
          sleep_status_pub.publish("SLEEP ACTIVATED")
          rospy.sleep(3) # Give Web page time to update.
          own_util.HostShutdown()

        # Read battery status from Atmega328P
        i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 100)  # Command to indicate a read is going to follow.
        i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 128)  # 128 means read battery status.
        rospy.sleep(i2c.i2cDelay) # Give Atmega328P time to process.
        battery_voltage = i2c.read_byte(i2c.slaveAddressAtmega328P, 0)
        battery_voltage = battery_voltage * 7.8 * 1.1 / 256
        battery_status_pub.publish("{:.2f}".format(battery_voltage) + " V")

        # Read solar panel status from Atmega328P
        i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 100)  # Command to indicate a read is going to follow.
        i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 129)  # 129 means read solar panel status.
        rospy.sleep(i2c.i2cDelay) # Give Atmega328P time to process.
        solarpanel_voltage = i2c.read_byte(i2c.slaveAddressAtmega328P, 0)
        solarpanel_voltage = solarpanel_voltage * 21.5 * 1.1 / 256
        # ReneB: To calculate the solar panel charging current we first calculate the voltage over the 10 ohm resistor as being
        # solarpanel_voltage - battery_voltage minus the voltage over the transistor (TIP32C) and diode (1N4002).
        # The voltage over the transistor and diode will be around 0.7 V.
        # This calculation is valid between 0 mA end the currents source current which is 60 mA.
        solarpanel_current = min(max((solarpanel_voltage - battery_voltage - 0.7) / 0.01, 0), 60)
        solarpanel_status_pub.publish("{:.2f}".format(solarpanel_voltage) + " V" + ", " + "{:.2f}".format(solarpanel_current) + " mA")

        wifi_status = own_util.HostGetWifiStatus()
        wifi_status_pub.publish(wifi_status)

        rospy.sleep(1)

    #rospy.spin()  # ReneB: Commented out because of while loop with sleep above.
