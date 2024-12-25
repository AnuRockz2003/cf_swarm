#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from builtin_interfaces.msg import Duration

class LeaderNode(Node):
    def __init__(self):
        super().__init__("leader_node")
        self.publisher_setpoint = self.create_publisher(Twist, '/cf1/cmd_vel', 10)
        self.publisher_pose = self.create_publisher(PoseStamped, '/cf1/leader_pose', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
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

    def initiate_landing(self):
        # Add the landing logic for the leader
        land_req = Land.Request()
        land_req.height = 0.1
        land_req.duration = Duration(sec=2, nanosec=0)
        self.land_client.call_async(land_req)
        self.get_logger().info("Leader landing initiated.")


class FollowerNode(Node):
    def __init__(self, follower_id):
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

        self.initial_offset = (0.0, 0.0, 0.0)  # Initialize offset as zero
        self.has_offset = False  # Flag to check if offset has been set

    def leader_pose_callback(self, msg):
        if not self.has_offset:
            # Calculate initial offset based on the leader's pose
            self.initial_offset = (
                msg.pose.position.x - self.get_initial_position()[0],
                msg.pose.position.y - self.get_initial_position()[1],
                msg.pose.position.z - self.get_initial_position()[2],
            )
            self.has_offset = True
            self.get_logger().info(f'Initial offset set for Follower: {self.initial_offset}')

        follower_twist = Twist()
        # Calculate desired position based on leader's position and follower's offset
        follower_twist.linear.x = msg.pose.position.x + self.initial_offset[0]
        follower_twist.linear.y = msg.pose.position.y + self.initial_offset[1]
        follower_twist.linear.z = msg.pose.position.z + self.initial_offset[2]

        self.publisher_setpoint.publish(follower_twist)
        self.get_logger().info(f'Follower setpoint published: {follower_twist.linear.x}, {follower_twist.linear.y}, {follower_twist.linear.z}')

    def get_initial_position(self):
        # This method should return the initial position of the follower drone.
        return (0.0, 0.0, 0.0)  # Replace this with actual logic to get initial position

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

    # Create leader node
    leader_node = LeaderNode()
    
    # Create follower nodes
    follower1 = FollowerNode(2)  # Follower 1
    follower2 = FollowerNode(3)  # Follower 2
    
    # Initiate takeoff for both followers
    follower1.initiate_takeoff()
    follower2.initiate_takeoff()

    try:
        rclpy.spin(leader_node)
    except KeyboardInterrupt:
        # Initiate landing for the leader and followers when interrupted
        leader_node.initiate_landing()
        follower1.initiate_landing()
        follower2.initiate_landing()
    finally:
        leader_node.destroy_node()
        follower1.destroy_node()
        follower2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()