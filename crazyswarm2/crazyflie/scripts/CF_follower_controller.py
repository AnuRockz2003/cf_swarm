#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from builtin_interfaces.msg import Duration

class FollowerNode(Node):
    def __init__(self, follower_id, initial_offset):
        super().__init__(f'follower_controller_{follower_id}')
        self.declare_parameter('robot_prefix', f'/cf{follower_id}')  # Set robot prefix for each follower
        self.hover_height = 0.05  # Adjust hover height as needed

        self.takeoff_client = self.create_client(Takeoff, self.get_parameter('robot_prefix').value + '/takeoff')
        self.land_client = self.create_client(Land, self.get_parameter('robot_prefix').value + '/land')
        self.notify_client = self.create_client(NotifySetpointsStop, self.get_parameter('robot_prefix').value + '/notify_setpoints_stop')

        self.subscription = self.create_subscription(
            PoseStamped,
            '/cf1/leader_pose',  # Subscribe to leader's pose
            self.leader_pose_callback,
            10
        )
        
        self.publisher_setpoint = self.create_publisher(Twist, self.get_parameter('robot_prefix').value + '/cmd_vel', 10)

        self.initial_offset = initial_offset  # Offset for follower position (x, y, z)

    def leader_pose_callback(self, msg):
        follower_twist = Twist()
        # Calculate desired position based on leader's position and follower's offset
        follower_twist.linear.x = msg.pose.position.x + self.initial_offset[0]
        follower_twist.linear.y = msg.pose.position.y + self.initial_offset[1]
        follower_twist.linear.z = msg.pose.position.z + self.initial_offset[2]

        self.publisher_setpoint.publish(follower_twist)
        self.get_logger().info(f'Follower setpoint published: {follower_twist.linear.x}, {follower_twist.linear.y}, {follower_twist.linear.z}')

    def initiate_takeoff(self):
        takeoff_req = Takeoff.Request()
        takeoff_req.height = self.hover_height
        takeoff_req.duration = Duration(sec=2, nanosec=0)
        self.takeoff_client.call_async(takeoff_req)

    def initiate_landing(self):
        notify_stop_req = NotifySetpointsStop.Request()
        self.notify_client.call_async(notify_stop_req)

        land_req = Land.Request()
        land_req.height = 0.1
        land_req.duration = Duration(sec=2, nanosec=0)
        self.land_client.call_async(land_req)

def main(args=None):
    rclpy.init(args=args)
    
    # Create each follower with unique initial offsets to maintain a formation
    follower1 = FollowerNode(2, (1.0, 0.0, 0.0))  # Follower 1, offset (1,0,0)
    follower2 = FollowerNode(3, (0.0, 1.0, 0.0))  # Follower 2, offset (0,1,0)
    
    try:
        follower1.initiate_takeoff()
        follower2.initiate_takeoff()
        rclpy.spin(follower1)
    except KeyboardInterrupt:
        follower1.initiate_landing()
        follower2.initiate_landing()
    finally:
        follower1.destroy_node()
        follower2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()