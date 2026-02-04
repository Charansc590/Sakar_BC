import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial


class ESP32UARTNode(Node):

    def __init__(self):
        super().__init__('esp32_uart_node')

        # Publishers
        self.ul_pub = self.create_publisher(
            Float32MultiArray, '/ul_data', 10)

        self.ir_pub = self.create_publisher(
            Float32MultiArray, '/ir_data', 10)

        # Serial port (USB UART)
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',   # change if needed
            baudrate=115200,
            timeout=1
        )

        self.get_logger().info('ESP32 UART node started')

        # Timer (20 Hz read rate)
        self.timer = self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return

        try:
            data = {}
            parts = line.split(',')

            for part in parts:
                key, value = part.split('=')
                data[key] = float(value)

            # Ultrasonic data
            ul_msg = Float32MultiArray()
            ul_msg.data = [data['US1'], data['US2']]
            self.ul_pub.publish(ul_msg)

            # IR data
            ir_msg = Float32MultiArray()
            ir_msg.data = [data['IR1'], data['IR2']]
            self.ir_pub.publish(ir_msg)

        except Exception as e:
            self.get_logger().warn(f'Parse error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ESP32UARTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
