import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import paho.mqtt.client as mqtt
import json
import math

def quaternion_from_yaw(yaw):
    """Yaw(라디안) → 쿼터니언 변환"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

class MqttNav2Goal(Node):
    def __init__(self):
        super().__init__('mqtt_nav2_goal')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # MQTT 설정
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        # MQTT 브로커 연결
        broker = "localhost"  # 필요에 맞게 변경
        port = 1883
        self.mqtt_client.connect(broker, port, 60)

        # MQTT 수신 루프를 비동기로 실행
        self.mqtt_client.loop_start()

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT 연결 성공, 코드 {rc}")
        # 예: robot/goal_pose 토픽 구독
        client.subscribe("robot/goal_pose")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            x = data.get("x")
            y = data.get("y")
            yaw = data.get("yaw", 0.0)
            frame_id = data.get("frame_id", "map")

            self.get_logger().info(f"MQTT Goal 수신: x={x}, y={y}, yaw={yaw}")
            self.send_goal(x, y, yaw, frame_id)
        except Exception as e:
            self.get_logger().error(f"MQTT 메시지 처리 오류: {e}")

    def send_goal(self, x, y, yaw, frame_id="map"):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 navigate_to_pose 액션 서버가 없음")
            return

        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        q = quaternion_from_yaw(yaw)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        goal_msg.pose = goal_pose

        self.get_logger().info(f"Nav2 Goal 전송: {goal_msg}")
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MqttNav2Goal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
