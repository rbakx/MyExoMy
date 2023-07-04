#!/usr/bin/env python
from exomy_msgs.msg import RoverCommand, MotorCommands
import rclpy
from rclpy.node import Node
from .rover import Rover
from std_msgs.msg import String
import time
from . import i2c
from . import own_util


class RobotNode(Node):
    def __init__(self):
        self.node_name = 'robot_node'
        super().__init__(self.node_name)

        self.joy_sub = self.create_subscription(
            RoverCommand,
            '/exomy_pkg/rover_command',
            self.joy_callback,
            10)

        # ReneB: subscribe to own_button topic, meant for buttons on the web page to turn the lights on and go to sleep.
        self.joy_sub = self.create_subscription(
            String,
            '/own_button',
            self.own_button_callback,
            10)

        self.robot_pub = self.create_publisher(
            MotorCommands,
            '/exomy_pkg/motor_commands',
            1)

        # ReneB: Added publishers for status.
        self.battery_status_pub = self.create_publisher(
            String,
            '/battery_status',
            1)
        self.solarpanel_status_pub = self.create_publisher(
            String,
            '/solarpanel_status',
            1)
        self.wifi_status_pub = self.create_publisher(
            String,
            '/wifi_status',
            1)
        self.sleep_status_pub = self.create_publisher(
            String,
            '/sleep_status',
            1)

        self.robot = Rover()

        # ReneB: Create Watchdog timer to send status every second.
        self.watchdog_timer = self.create_timer(1, self.watchdog)

        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))


    def joy_callback(self, msg):
        cmds = MotorCommands()

        self.robot.setLocomotionMode(msg.locomotion_mode)
        cmds.motor_angles = self.robot.joystickToSteeringAngle(
            msg.vel, msg.steering)
        cmds.motor_speeds = self.robot.joystickToVelocity(
            msg.vel, msg.steering)

        self.robot_pub.publish(cmds)


    def own_button_callback(self, msg):
        if msg.data == "lights_on":
            i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 1)  # Turn lights on
        elif msg.data == "lights_off":
            i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 2)  # Turn lights off
        elif msg.data == "goto_sleep":
            i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 42)  # Send sleep command
            # After sending the sleep command to the Atmega328P, the Atmega328P will send back '42' as hardware status indicating it will put the MyExoMy to sleep after a delay.
            # This hardware status is read below so the Raspberry Pi can properly shut down.


    def watchdog(self):
        msg = String()
        # Read hardware status from Atmega328P
        i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 100)  # Command to indicate a read is going to follow.
        i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 255)  # 255 means read hardware status.
        time.sleep(i2c.i2cDelay) # Give Atmega328P time to process.
        hardware_status = i2c.read_byte(i2c.slaveAddressAtmega328P, 0)
        # A hardware status of 42 means that the Atmega328P received a request to put the MyExoMy to sleep after a delay.
        # This sleep request can come from the Web Interface (RaspBerry Pi) and also from the Atmega328P itself when there is no motion (activity) detected for a certain period of time.
        if hardware_status == 42:
            # ReneB: If we get here it means that the ATmega328P is going to put the MyExoMy to sleep after a short waiting period.
            # After this short period the ATmega328P will switch off the power to the Raspberry Pi. During this period we shutdown the Raspberry Pi in a proper way.
            msg.data = "SLEEP ACTIVATED" 
            self.sleep_status_pub.publish(msg)
            time.sleep(3) # Give Web page time to update.
            own_util.Shutdown()

        # Read battery status from Atmega328P
        i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 100)  # Command to indicate a read is going to follow.
        i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 128)  # 128 means read battery status.
        time.sleep(i2c.i2cDelay) # Give Atmega328P time to process.
        battery_voltage = i2c.read_byte(i2c.slaveAddressAtmega328P, 0)
        battery_voltage = battery_voltage * 7.8 * 1.1 / 256
        msg.data = "{:.2f}".format(battery_voltage) + " V"
        self.battery_status_pub.publish(msg)

        # Read solar panel status from Atmega328P
        i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 100)  # Command to indicate a read is going to follow.
        i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 129)  # 129 means read solar panel status.
        time.sleep(i2c.i2cDelay) # Give Atmega328P time to process.
        solarpanel_voltage = i2c.read_byte(i2c.slaveAddressAtmega328P, 0)
        solarpanel_voltage = solarpanel_voltage * 21.5 * 1.1 / 256
        # ReneB: To calculate the solar panel charging current we first calculate the voltage over the 10 ohm resistor as being
        # solarpanel_voltage - battery_voltage minus the voltage over the transistor (TIP32C) and diode (1N4002).
        # The voltage over the transistor and diode will be around 0.7 V.
        # This calculation is valid between 0 mA and the current-source current which is 60 mA.
        solarpanel_current = min(max((solarpanel_voltage - battery_voltage - 0.7) / 0.01, 0), 60)
        msg.data = "{:.2f}".format(solarpanel_voltage) + " V" + ", " + "{:.2f}".format(solarpanel_current) + " mA"
        self.solarpanel_status_pub.publish(msg)

        msg.data = own_util.GetWifiStatus()
        self.wifi_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        robot_node = RobotNode()
        try:
            rclpy.spin(robot_node)
        finally:
            robot_node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
