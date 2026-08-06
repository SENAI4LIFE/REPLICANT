#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

MIN_DISTANCE_M = 0.02
MAX_POSES = 20000


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.declare_parameter('use_sim_time', True)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.path = Path()
        self.last_x = None
        self.last_y = None

        self.pub = self.create_publisher(Path, '/trajectory', qos)
        self.sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is not None:
            moved = math.hypot(x - self.last_x, y - self.last_y)
            if moved < MIN_DISTANCE_M:
                return

        self.last_x = x
        self.last_y = y

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.header = msg.header
        self.path.poses.append(pose)

        if len(self.path.poses) > MAX_POSES:
            self.path.poses.pop(0)

        self.pub.publish(self.path)


def main():
    rclpy.init()
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
