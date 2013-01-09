#!/usr/bin/env python
# -*- coding: utf-8 -*-

from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_imu import IMU

import math
import time

class Q:
    HOST = "localhost"
    PORT = 4223
    UID = "aeoUQwwyAvY" # Change to your UID

    def __init__(self):
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_z = 0.0
        self.base_w = 0.0

        self.imu = IMU(self.UID) # Create device object
        self.ipcon = IPConnection(self.HOST, self.PORT) # Create IPconnection to brickd
        self.ipcon.add_device(self.imu) # Add device to IP connection
        # Don't use device before it is added to a connection

        # Wait for IMU to settle
        print 'Set IMU to base position and wait for 10 seconds'
        print 'Base position will be 0 for all angles'
        time.sleep(10)
        q = self.imu.get_quaternion()
        self.set_base_coordinates(q.x, q.y, q.z, q.w)

        # Set period for quaternion callback to 10ms
        self.imu.set_quaternion_period(10)

        # Register quaternion callback
        self.imu.register_callback(self.imu.CALLBACK_QUATERNION, self.quaternion_cb)

    def quaternion_cb(self, x, y, z, w):
        # Use conjugate of quaternion to rotate coordinates according to base system
        x, y, z, w = self.make_relative_coordinates(-x, -y, -z, w)

        x_angle = int(math.atan2(2.0*(y*z - w*x), 1.0 - 2.0*(x*x + y*y))*180/math.pi)
        y_angle = int(math.atan2(2.0*(x*z + w*y), 1.0 - 2.0*(x*x + y*y))*180/math.pi)
        z_angle = int(math.atan2(2.0*(x*y + w*z), 1.0 - 2.0*(x*x + z*z))*180/math.pi)

        print 'x: {0}, y: {1}, z: {2}'.format(x_angle, y_angle, z_angle)

    def set_base_coordinates(self, x, y, z, w):
        self.base_x = x
        self.base_y = y
        self.base_z = z
        self.base_w = w

    def make_relative_coordinates(self, x, y, z, w):
        # Multiply base quaternion with current quaternion
        return (
            w * self.base_x + x * self.base_w + y * self.base_z - z * self.base_y,
            w * self.base_y - x * self.base_z + y * self.base_w + z * self.base_x,
            w * self.base_z + x * self.base_y - y * self.base_x + z * self.base_w,
            w * self.base_w - x * self.base_x - y * self.base_y - z * self.base_z
        )

if __name__ == "__main__":
    q = Q()

    raw_input('Press key to exit\n') # Use input() in Python 3
    q.ipcon.destroy()
