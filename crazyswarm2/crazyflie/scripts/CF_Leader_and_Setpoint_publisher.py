#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped

timer_period = 1.0  # Leader updates every second

class SetpointPublisher(Node):
    def __init__(self):
        super().__init__("setpoint_publisher")
        self.publisher_setpoint = self.create_publisher(Twist, '/cf1/cmd_vel', 10)
        self.publisher_pose = self.create_publisher(PoseStamped, '/cf1/leader_pose', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.index = 0
        self.sequence = [
            (0.0, 0.0, 0.3),  # Setpoints for the leader's movement
            (0.0, 0.0, 0.2),
            (0.2, 0.1, 0.3),
            (0.2, 0.3, 0.2),
            (-0.2, 0.2, 0.2),
            (-0.2, 0.0, 0.3),
            (0.0, 0.0, 0.4),
            (0.0, 0.0, 0.1),
        ]

    def timer_callback(self):
        if self.index < len(self.sequence):
            setpoint_msg = Twist()
            setpoint_msg.linear.x = self.sequence[self.index][0]
            setpoint_msg.linear.y = self.sequence[self.index][1]
            setpoint_msg.linear.z = self.sequence[self.index][2]
            self.publisher_setpoint.publish(setpoint_msg)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = setpoint_msg.linear.x
            pose_msg.pose.position.y = setpoint_msg.linear.y
            pose_msg.pose.position.z = setpoint_msg.linear.z
            self.publisher_pose.publish(pose_msg)

            self.index += 1
            self.get_logger().info(f'Leader setpoint {self.index} published: {setpoint_msg.linear.x}, {setpoint_msg.linear.y}, {setpoint_msg.linear.z}')
        else:
            setpoint_msg = Twist()
            setpoint_msg.linear.x = 0.0
            setpoint_msg.linear.y = 0.0
            setpoint_msg.linear.z = 0.0
            self.publisher_setpoint.publish(setpoint_msg)
            self.get_logger().info("All setpoints sent, stopping...")
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = SetpointPublisher()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()