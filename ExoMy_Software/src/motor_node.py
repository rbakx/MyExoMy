#!/usr/bin/env python
import time
import rospy
import i2c

from exomy.msg import MotorCommands
from motors import Motors

motors = Motors()
global watchdog_timer


def callback(cmds):
    motors.setSteering(cmds.motor_angles)
    motors.setDriving(cmds.motor_speeds)

    global watchdog_timer
    watchdog_timer.shutdown()
    # If this timer runs longer than the duration specified,
    # then watchdog() is called stopping the driving motors.
    # ReneB: Decreased watchdog timeout from 5 seconds to 0.5 second.
    #        This to make sure that if the Wifi connection drops the ExoMy stops in 0.5 second.
    #        It still is possible that after the ExoMy stops it starts moving again due to delayed Wifi packet delivery.
    watchdog_timer = rospy.Timer(rospy.Duration(0.5), watchdog, oneshot=True)


def shutdown():
    motors.stopMotors()


def watchdog(event):
    rospy.loginfo("Watchdog fired. Stopping driving motors.")
    motors.stopMotors()
    i2c.write_byte(i2c.slaveAddressAtmega328P, 0, 10)  # Send motion command to Arduino to prevent putting MyExoMy to sleep.


if __name__ == "__main__":
    # This node waits for commands from the robot and sets the motors accordingly
    rospy.init_node("motors")
    rospy.loginfo("Starting the motors node")
    rospy.on_shutdown(shutdown)

    global watchdog_timer
    watchdog_timer = rospy.Timer(rospy.Duration(1.0), watchdog, oneshot=True)

    sub = rospy.Subscriber(
        "/motor_commands", MotorCommands, callback, queue_size=1)

    rate = rospy.Rate(10)

    rospy.spin()
