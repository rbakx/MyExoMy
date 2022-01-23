#!/usr/bin/env/python
import smbus
import rclpy

# Global variables
# ReneB: I2C address of Atmega328P.
slaveAddressAtmega328P = 0x04
# i2cDelay is the minimum time needed between two I2C commands to give the Atmega328P time to handle the commands.
# The Atmega328P does not buffer the I2C commands. If the Raspberry issues an I2C command too fast for the Atmega328P,
# it will overwrite the previous command on the Atmega328P.
i2cDelay = 0.1


def get_smbus():
    try:
        bus = smbus.SMBus(1)
        return bus
    except Exception as e:
        log_node = rclpy.create_node('log_node')
        log_node.get_logger().info('I2C get_smbus exception: ' + str(e))
        return None


def read_byte(slaveAddr, adr):
    try:
        bus = get_smbus()
        byte = bus.read_byte_data(slaveAddr, adr)
        return byte
    except Exception as e:
        log_node = rclpy.create_node('log_node')
        log_node.get_logger().info('I2C read_byte exception: ' + str(e))
        return 0

def read_word(slaveAddr, adr):
    try:
        bus = get_smbus()
        high = bus.read_byte_data(slaveAddr, adr)
        low = bus.read_byte_data(slaveAddr, adr+1)
        val = (high << 8) + low
        return val
    except Exception as e:
        log_node = rclpy.create_node('log_node')
        log_node.get_logger().info('I2C read_word exception: ' + str(e))
        return 0

def read_word_2c(slaveAddr, adr):
    try:
        val = read_word(slaveAddr, adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val
    except Exception as e:
        log_node = rclpy.create_node('log_node')
        log_node.get_logger().info('I2C read_word_2c exception: ' + str(e))
        return 0

def write_byte(slaveAddr, adr, value):
    try:
        bus = get_smbus()
        bus.write_byte_data(slaveAddr, adr, value)
    except Exception as e:
        log_node = rclpy.create_node('log_node')
        log_node.get_logger().info('I2C write_byte exception: ' + str(e))
        return 0
