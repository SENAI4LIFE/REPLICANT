import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

TIBIA_JOINTS = ['tibia1', 'tibia2', 'tibia3', 'tibia4', 'tibia5', 'tibia6']

SPRING_STIFFNESS   = 1000.0
CONTACT_THRESHOLD  = 2.0

class ContactMonitor(Node):
    def __init__(self):
        super().__init__('contact_monitor')

        self.force_pub = self.create_publisher(
            Float64MultiArray, '/tiffany/leg_forces', 10)

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'ContactMonitor active. '
            f'Stiffness={SPRING_STIFFNESS} N/m  '
            f'Threshold={CONTACT_THRESHOLD} N'
        )

    def _joint_state_cb(self, msg: JointState):
        joint_map = dict(zip(msg.name, msg.position))

        forces       = []
        display_parts = []

        for i, joint_name in enumerate(TIBIA_JOINTS):
            leg_num = i + 1

            if joint_name in joint_map:
                raw_force = abs(joint_map[joint_name]) * SPRING_STIFFNESS
                force     = max(0.0, raw_force)
                contact   = 1 if force >= CONTACT_THRESHOLD else 0
            else:
                force   = 0.0
                contact = -1

            forces.append(force)
            status_str = str(contact) if contact >= 0 else 'ERR'
            display_parts.append(f'L{leg_num}:{status_str}')

        out      = Float64MultiArray()
        out.data = forces
        self.force_pub.publish(out)

        sys.stdout.write(f"\r{'  '.join(display_parts)}  ")
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = ContactMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print('\nContactMonitor stopped.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()