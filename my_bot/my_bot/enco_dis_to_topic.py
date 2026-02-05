#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import serial
import math
import time

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler


class SerialToROS(Node):

    def __init__(self):
        super().__init__('serial_to_ros')

        # ---------- SERIAL ----------
        self.port = '/dev/ttyUSB0'   # change if needed
        self.baud = 115200
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)

        # ---------- ROBOT PARAMS ----------
        self.wheel_radius = 0.09          # meters (90 mm)
        self.wheel_base = 0.30            # meters (CHANGE to your robot)
        self.ticks_per_rev = 600          # CHANGE to your encoder

        self.dist_per_tick = 2.0 * math.pi * self.wheel_radius / self.ticks_per_rev

        # ---------- STATE ----------
        self.prev_encL = None
        self.prev_encR = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

        # ---------- ROS PUB ----------
        self.dis_pub = self.create_publisher(
            Float32MultiArray, '/dis_data', 10
        )

        self.odom_pub = self.create_publisher(
            Odometry, '/odom_encoder', 10
        )

        self.timer = self.create_timer(0.02, self.read_serial)  # 50 Hz

        self.get_logger().info("Serial → ROS node started")

    def read_serial(self):
        if not self.ser.in_waiting:
            return

        try:
            line = self.ser.readline().decode().strip()
            parts = line.split()

            if len(parts) != 6:
                return

            encL, encR, ir1, ir2, us1, us2 = map(float, parts)

            # ---------- DIS DATA ----------
            dis_msg = Float32MultiArray()
            dis_msg.data = [ir1, ir2, us1, us2]
            self.dis_pub.publish(dis_msg)

            # ---------- ODOM ----------
            if self.prev_encL is None:
                self.prev_encL = encL
                self.prev_encR = encR
                self.last_time = time.time()
                return

            d_encL = encL - self.prev_encL
            d_encR = encR - self.prev_encR

            self.prev_encL = encL
            self.prev_encR = encR

            dl = d_encL * self.dist_per_tick
            dr = d_encR * self.dist_per_tick

            dc = (dl + dr) / 2.0
            dtheta = (dr - dl) / self.wheel_base

            self.theta += dtheta
            self.x += dc * math.cos(self.theta)
            self.y += dc * math.sin(self.theta)

            now = time.time()
            dt = now - self.last_time
            self.last_time = now

            vx = dc / dt if dt > 0 else 0.0
            vth = dtheta / dt if dt > 0 else 0.0

            q = quaternion_from_euler(0.0, 0.0, self.theta)

            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = Quaternion(
                x=q[0], y=q[1], z=q[2], w=q[3]
            )

            odom.twist.twist.linear.x = vx
            odom.twist.twist.angular.z = vth

            self.odom_pub.publish(odom)

        except Exception as e:
            self.get_logger().warn(f"Serial parse error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialToROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
