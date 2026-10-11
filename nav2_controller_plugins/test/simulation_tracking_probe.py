# 261011 수정: Gazebo/Nav2 액션 결과, 실제 odom 및 map TF로 추종을 검증하는 실행 도구.
import math
import time
import json
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState  # 261011 수정: 비활성 BT 서버에 목표를 보내는 시작 경쟁을 방지한다.
from tf2_ros import Buffer, TransformListener


class Probe(Node):
    # 261011 수정: 명령값뿐 아니라 시뮬레이터 이동과 RViz가 사용하는 TF를 함께 측정한다.
    def __init__(self):
        super().__init__('tracking_verification_261011')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.map = None
        self.global_map = None  # 261011 수정: 실시간 lidar 장애물도 목표 선택에 반영한다.
        self.path = None
        self.odom = None
        self.commands = []
        self.samples = []
        self.cross_track = []  # 261011 수정: 최근 발행된 전역경로 대비 위치 오차도 기록한다.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', lambda msg: setattr(self, 'map', msg), qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda msg: setattr(self, 'global_map', msg), qos)
        self.create_subscription(Path, '/plan', self.plan, 10)
        self.create_subscription(Odometry, '/odom', lambda msg: setattr(self, 'odom', msg), 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.command, 10)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.state_client = self.create_client(GetState, '/bt_navigator/get_state')

    def command(self, message):
        self.commands.append((message.twist.linear.x, message.twist.angular.z))

    def plan(self, message):
        # 261011 수정: 실패 시 실제 계획의 전후진 cusp와 곡선을 분석할 수 있도록 기록한다.
        self.path = message
        print('PLAN', json.dumps([[p.pose.position.x, p.pose.position.y,
              math.atan2(2*p.pose.orientation.w*p.pose.orientation.z,
                         1-2*p.pose.orientation.z*p.pose.orientation.z)] for p in message.poses]), flush=True)

    def pose(self):
        try:
            transform = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = transform.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            return (transform.transform.translation.x, transform.transform.translation.y, yaw)
        except Exception:
            return None

    def clear(self, x, y):
        for grid in (self.map, self.global_map):
            if grid is None:
                return False
            # 261011 수정: global_costmap은 이미 robot_radius만큼 팽창하므로 중복 팽창하지 않는다.
            offsets = (0.0,) if grid is self.global_map else (-0.22, 0.0, 0.22)
            for dx in offsets:
                for dy in offsets:
                    ix = int((x + dx - grid.info.origin.position.x) / grid.info.resolution)
                    iy = int((y + dy - grid.info.origin.position.y) / grid.info.resolution)
                    if not (0 <= ix < grid.info.width and 0 <= iy < grid.info.height):
                        return False
                    value = grid.data[iy * grid.info.width + ix]
                    if value < 0 or value >= 99:
                        return False
        return True

    def pump(self, duration):
        finish = time.monotonic() + duration
        while time.monotonic() < finish:
            rclpy.spin_once(self, timeout_sec=0.05)

    # 261011 수정: 직선으로 안전한 목표를 선택하고 액션 종료 및 실제 변위를 기록한다.
    def run(self, initialize, target):
        # 261011 수정: 각 목표의 명령/경로 오차를 독립적으로 집계한다.
        self.commands, self.samples, self.cross_track = [], [], []
        self.path = None
        if initialize:
            deadline = time.monotonic() + 60
            while self.initial.get_subscription_count() == 0 or self.get_clock().now().nanoseconds == 0:
                if time.monotonic() > deadline:
                    raise RuntimeError('AMCL initialpose subscriber or simulation clock not ready')
                self.pump(0.2)
            for _ in range(3):
                message = PoseWithCovarianceStamped()
                message.header.frame_id = 'map'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose.position.x = 2.0
                message.pose.pose.orientation.w = 1.0
                message.pose.covariance[0] = message.pose.covariance[7] = 0.04
                message.pose.covariance[35] = 0.04
                self.initial.publish(message)
                self.pump(2)
        deadline = time.monotonic() + 60
        while not (self.pose() and self.map and self.global_map and self.client.server_is_ready()):
            if time.monotonic() > deadline:
                raise RuntimeError('AMCL/map/action server not ready')
            self.pump(0.2)
        # 261011 수정: 액션 서버 존재 여부뿐 아니라 lifecycle ACTIVE 상태까지 기다린다.
        while time.monotonic() < deadline:
            if not self.state_client.service_is_ready():
                self.pump(0.2)
                continue
            state = self.state_client.call_async(GetState.Request())
            while not state.done() and time.monotonic() < deadline:
                self.pump(0.1)
            if state.done() and state.result().current_state.id == 3:
                break
            self.pump(0.2)
        else:
            raise RuntimeError('BT navigator lifecycle is not ACTIVE')
        start = self.pose()
        if target is None:
            for angle in (0, math.pi/2, -math.pi/2, math.pi):
                candidate = (start[0] + math.cos(angle), start[1] + math.sin(angle),
                             start[2] if angle == math.pi else angle)
                if all(self.clear(start[0] + (candidate[0]-start[0])*i/20,
                                  start[1] + (candidate[1]-start[1])*i/20) for i in range(21)):
                    target = candidate
                    break
        if target is None:
            raise RuntimeError('No clear 1m straight goal near spawn')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = target[:2]
        goal.pose.pose.orientation.z = math.sin(target[2]/2)
        goal.pose.pose.orientation.w = math.cos(target[2]/2)
        print('START', json.dumps({'pose': start, 'goal': target}), flush=True)
        request = self.client.send_goal_async(goal)
        while not request.done():
            self.pump(0.1)
        handle = request.result()
        if not handle.accepted:
            raise RuntimeError('Goal rejected')
        result = handle.get_result_async()
        deadline = time.monotonic() + 180
        next_print = time.monotonic()
        while not result.done() and time.monotonic() < deadline:
            self.pump(0.1)
            pose = self.pose()
            if pose:
                self.samples.append(pose)
                if self.path and len(self.path.poses) > 1:
                    distances = []
                    for a, b in zip(self.path.poses, self.path.poses[1:]):
                        ax, ay = a.pose.position.x, a.pose.position.y
                        dx, dy = b.pose.position.x-ax, b.pose.position.y-ay
                        t = max(0.0, min(1.0, ((pose[0]-ax)*dx+(pose[1]-ay)*dy) / max(1e-12, dx*dx+dy*dy)))
                        distances.append(math.hypot(pose[0]-ax-t*dx, pose[1]-ay-t*dy))
                    self.cross_track.append(min(distances))
            if time.monotonic() >= next_print:
                print('TRACK', json.dumps({'pose': pose, 'odom_v': self.odom.twist.twist.linear.x if self.odom else None,
                      'cmd': self.commands[-1] if self.commands else None}), flush=True)
                next_print += 2
        if not result.done():
            cancel = handle.cancel_goal_async()
            self.pump(3)
            raise RuntimeError('Navigation timed out; cancellation requested')
        final = self.pose()
        data = {'status': result.result().status, 'start': start, 'goal': target, 'final': final,
                'position_error': math.hypot(final[0]-target[0], final[1]-target[1]),
                'yaw_error': abs(math.atan2(math.sin(final[2]-target[2]), math.cos(final[2]-target[2]))),
                'negative_commands': sum(v < -0.01 for v, _ in self.commands),
                'max_command_v': max((abs(v) for v, _ in self.commands), default=0),
                'max_cross_track': max(self.cross_track, default=None),
                'mean_cross_track': sum(self.cross_track)/len(self.cross_track) if self.cross_track else None,
                'samples': len(self.samples)}
        print('RESULT', json.dumps(data), flush=True)
        return data['status'] == 4


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--initialize', action='store_true')
    parser.add_argument('--goal', type=float, nargs=3)
    parser.add_argument('--suite', action='store_true')  # 261011 수정: 전후진과 방향 전환을 연속 검증한다.
    arguments = parser.parse_args()
    rclpy.init()
    probe = Probe()
    try:
        if arguments.suite:
            success = probe.run(True, (1.0, 0.0, 0.0))
            if success:
                success = probe.run(False, (2.0, 0.0, 0.0))
            if success:
                success = probe.run(False, (1.5, 0.5, math.pi/2))
        else:
            success = probe.run(arguments.initialize, arguments.goal)
    finally:
        probe.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)
