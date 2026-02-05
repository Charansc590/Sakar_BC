#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from smbus2 import SMBus   # ✅ CORRECT FOR UBUNTU


class FilteredMPU6050Node(Node):
    def __init__(self):
        super().__init__('filtered_mpu6050_node')

        self.get_logger().info(
            'Filtered MPU6050 IMU node (+X forward, +Y left, +Z up)'
        )

        # ---------- I2C ----------
        self.bus_num = 1
        self.addr = 0x68

        try:
            self.bus = SMBus(self.bus_num)

            # Wake up MPU6050
            self.bus.write_byte_data(self.addr, 0x6B, 0x00)

            # Accel ±2g
            self.bus.write_byte_data(self.addr, 0x1C, 0x00)

            # Gyro ±250 dps
            self.bus.write_byte_data(self.addr, 0x1B, 0x00)

            # DLPF ~42 Hz (reduces noise)
            self.bus.write_byte_data(self.addr, 0x1A, 0x03)

            time.sleep(0.1)

        except Exception as e:
            self.get_logger().error(f'I2C init failed: {e}')
            raise

        # ---------- offsets ----------
        self.ax_off = self.ay_off = self.az_off = 0.0
        self.gx_off = self.gy_off = self.gz_off = 0.0

        # ---------- moving average ----------
        self.WINDOW = 20
        self.buf_ax = []
        self.buf_ay = []
        self.buf_az = []
        self.buf_gx = []
        self.buf_gy = []
        self.buf_gz = []

        # ---------- ROS ----------
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------- calibrate ----------
        self.calibrate()

        # ---------- timers ----------
        self.timer_imu = self.create_timer(0.02, self.publish_imu)   # 50 Hz
        self.timer_tf = self.create_timer(0.05, self.broadcast_tf)  # 20 Hz

    # ---------- I2C helpers ----------
    def read_word(self, reg):
        high = self.bus.read_byte_data(self.addr, reg)
        low = self.bus.read_byte_data(self.addr, reg + 1)
        val = (high << 8) | low
        if val & 0x8000:
            val = -((65535 - val) + 1)
        return val

    # ---------- calibration ----------
    def calibrate(self):
        self.get_logger().info('Calibrating IMU... keep robot still (3s)')
        N = 300

        ax = ay = az = gx = gy = gz = 0.0

        for _ in range(N):
            ax += self.read_word(0x3B)
            ay += self.read_word(0x3D)
            az += self.read_word(0x3F)
            gx += self.read_word(0x43)
            gy += self.read_word(0x45)
            gz += self.read_word(0x47)
            time.sleep(0.01)

        self.ax_off = ax / N
        self.ay_off = ay / N
        self.az_off = az / N
        self.gx_off = gx / N
        self.gy_off = gy / N
        self.gz_off = gz / N

        self.get_logger().info(
            f'Offsets: ax={self.ax_off:.1f}, ay={self.ay_off:.1f}, '
            f'az={self.az_off:.1f}, gx={self.gx_off:.1f}, '
            f'gy={self.gy_off:.1f}, gz={self.gz_off:.1f}'
        )

    # ---------- publish IMU ----------
    def publish_imu(self):
        ax = self.read_word(0x3B) - self.ax_off
        ay = self.read_word(0x3D) - self.ay_off
        az = self.read_word(0x3F) - self.az_off
        gx = self.read_word(0x43) - self.gx_off
        gy = self.read_word(0x45) - self.gy_off
        gz = self.read_word(0x47) - self.gz_off

        self.buf_ax.append(ax)
        self.buf_ay.append(ay)
        self.buf_az.append(az)
        self.buf_gx.append(gx)
        self.buf_gy.append(gy)
        self.buf_gz.append(gz)

        if len(self.buf_ax) > self.WINDOW:
            self.buf_ax.pop(0)
            self.buf_ay.pop(0)
            self.buf_az.pop(0)
            self.buf_gx.pop(0)
            self.buf_gy.pop(0)
            self.buf_gz.pop(0)

        ax = sum(self.buf_ax) / len(self.buf_ax)
        ay = sum(self.buf_ay) / len(self.buf_ay)
        az = sum(self.buf_az) / len(self.buf_az)
        gx = sum(self.buf_gx) / len(self.buf_gx)
        gy = sum(self.buf_gy) / len(self.buf_gy)
        gz = sum(self.buf_gz) / len(self.buf_gz)

        # convert units
        ax *= 9.81 / 16384.0
        ay *= 9.81 / 16384.0
        az *= 9.81 / 16384.0
        gx *= math.pi / (180.0 * 131.0)
        gy *= math.pi / (180.0 * 131.0)
        gz *= math.pi / (180.0 * 131.0)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.orientation_covariance[0] = -1.0  # no orientation here

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        self.imu_pub.publish(msg)

    # ---------- TF ----------
    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.z = 0.10
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FilteredMPU6050Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
