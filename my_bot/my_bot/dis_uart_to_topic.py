import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial


class ESP32UARTNode(Node):

    def __init__(self):
        super().__init__('esp32_uart_node')

        # ROS publisher
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            '/dis_data',
            10
        )

        # UART config
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',   # change if needed
            baudrate=115200,
            timeout=1
        )

        # Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.read_uart)

        self.get_logger().info('ESP32 UART → /dis_data node started')

    def read_uart(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()

        if not line:
            return

        try:
            parts = line.split(',')
            data = {}

            for part in parts:
                key, value = part.split('=')
                data[key] = float(value)

            msg = Float32MultiArray()
            msg.data = [
                data['IR1'],
                data['IR2'],
                data['US1'],
                data['US2']
            ]

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Parse error: {line}')


def main(args=None):
    rclpy.init(args=args)
    node = ESP32UARTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
