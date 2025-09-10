import rclpy
from rclpy.node import Node
import serial

# cmd_val 메시지 타입에 맞게 변경
from std_msgs.msg import Int32  

class CmdValToUart(Node):
    def __init__(self):
        super().__init__('cmd_val_to_uart')

        # 파라미터 선언
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # UART 초기화
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"UART 포트 열림: {port} ({baudrate}bps)")
        except Exception as e:
            self.get_logger().error(f"UART 포트 열기 실패: {e}")
            raise

        # cmd_val 토픽 구독
        self.subscription = self.create_subscription(
            Int32,          # cmd_val 메시지 타입
            'cmd_val',      # 토픽 이름
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        data = f"{msg.data}\n"
        try:
            self.ser.write(data.encode('utf-8'))
            self.get_logger().info(f"UART 송신: {data.strip()}")
        except Exception as e:
            self.get_logger().error(f"UART 송신 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdValToUart()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
