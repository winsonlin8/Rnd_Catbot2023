#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import odrive
from odrive.enums import *
import time
import math
import fibre

class ODriveNode(Node):
    def __init__(self):
        super().__init__('odrive_node')

        # Find the connected ODrive
        self.odrv0 = self.find_odrive_by_serial("394d357c3231")

        # Calibrate motor and set to closed loop control mode
        self.setup_motor(self.odrv0.axis0)

        # Create subscriber for the motor
        self.velocity_subscriber_odrv0 = self.create_subscription(
            Int32,
            'motor_velocity_odrv0',
            lambda msg: self.velocity_callback(msg, self.odrv0.axis0),
            10)

    def find_odrive_by_serial(self, serial_number):
        context = fibre.USBContext()
        for device in context.get_device_list():
            if device.serial_number == serial_number:
                return odrive.find_any(serial_number=serial_number)
        self.get_logger().error(f"ODrive with serial {serial_number} not found")
        return None

    def setup_motor(self, axis):
        self.get_logger().info("Calibrating motor...")
        axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY
        self.get_logger().info("Motor setup complete.")

    def set_motor_velocity(self, axis, velocity):
        axis.controller.vel_setpoint = velocity

    def velocity_callback(self, msg, axis):
        self.get_logger().info(f"Setting velocity to {msg.data}")
        self.set_motor_velocity(axis, msg.data)

def main(args=None):
    rclpy.init(args=args)
    odrive_node = ODriveNode()
    rclpy.spin(odrive_node)

    # Clean up and shutdown
    odrive_node.set_motor_velocity(odrive_node.odrv0.axis0, 0)
    odrive_node.odrv0.axis0.requested_state = AXIS_STATE_IDLE
    odrive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
