import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistBridge(Node):
    def __init__(self):
        super().__init__('twist_bridge')
        # 1. Gazebo가 알아들을 수 있는 Twist 타입으로 발행 (Publish)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 2. Nav2가 뱉어내는 TwistStamped 타입을 구독 (Subscribe)
        self.sub = self.create_subscription(TwistStamped, '/cmd_vel', self.cb, 10)
        self.get_logger().info("🚀 TwistStamped -> Twist 변환 브릿지 가동 시작!")

    def cb(self, msg):
        out_msg = Twist()
        # 시간(Header) 껍데기는 버리고 알맹이(선속도, 각속도)만 복사
        out_msg.linear = msg.twist.linear
        out_msg.angular = msg.twist.angular
        self.pub.publish(out_msg)

def main():
    rclpy.init()
    rclpy.spin(TwistBridge())

if __name__ == '__main__':
    main()