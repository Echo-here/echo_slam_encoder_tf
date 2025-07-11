import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import random

class EncoderReaderNode(Node):
    def __init__(self):
        super().__init__('encoder_reader')
        self.pub = self.create_publisher(Int32MultiArray, 'encoder_counts', 10)

        # 엔코더 초기값 설정
        self.left_encoder = 0
        self.right_encoder = 0

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        # 가상 엔코더 값 업데이트 (랜덤값을 사용하여 엔코더의 회전 값을 시뮬레이션)
        left_delta = random.randint(0, 10)  # 왼쪽 바퀴 회전값 (가상)
        right_delta = random.randint(0, 10)  # 오른쪽 바퀴 회전값 (가상)

        self.left_encoder += left_delta
        self.right_encoder += right_delta

        # 엔코더 값을 ROS2 토픽에 발행
        counts = [self.left_encoder, self.right_encoder]
        msg = Int32MultiArray(data=counts)
        self.pub.publish(msg)

        self.get_logger().info(f"Published encoder counts: {counts}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
