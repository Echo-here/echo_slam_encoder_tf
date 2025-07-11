import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

class GazeboEncoderPublisher(Node):
    def __init__(self):
        super().__init__('gazebo_encoder_publisher')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.pub = self.create_publisher(Int32MultiArray, 'encoder_counts', 10)
        self.prev_left = 0.0
        self.prev_right = 0.0
        self.left_total = 0
        self.right_total = 0
        self.cpr = 500  # 엔코더 해상도

    def joint_callback(self, msg):
        try:
            left_idx = msg.name.index("left_wheel_joint")
            right_idx = msg.name.index("right_wheel_joint")
            left_pos = msg.position[left_idx]
            right_pos = msg.position[right_idx]

            delta_left = left_pos - self.prev_left
            delta_right = right_pos - self.prev_right
            self.prev_left = left_pos
            self.prev_right = right_pos

            # rad → pulse
            self.left_total += int(delta_left / (2 * 3.141592) * self.cpr)
            self.right_total += int(delta_right / (2 * 3.141592) * self.cpr)

            msg_out = Int32MultiArray()
            msg_out.data = [self.left_total, self.right_total]
            self.pub.publish(msg_out)

        except ValueError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = GazeboEncoderPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
