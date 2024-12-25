#!/usr/bin/env python3

"""
A Twist message handler that get incoming twist messages from 
    external packages and handles proper takeoff, landing and
    hover commands of connected crazyflie in the crazyflie_server
    node

    2022 - K. N. McGuire (Bitcraze AB)
"""
import rclpy
from rclpy.node import Node
import scipy.io
import numpy as np

from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import Bool
from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from crazyflie_interfaces.msg import Hover
import time

class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_cf')
        self.declare_parameter('hover_height', 0.1)
        self.declare_parameter('robot_prefix', '/cf')
        self.declare_parameter('leader_robot_prefix', '/cf1')

        self.hover_height  = self.get_parameter('hover_height').value
        robot_prefix  = self.get_parameter('robot_prefix').value
        leader_robot_prefix  = self.get_parameter('leader_robot_prefix').value
        self.z_pos = []
        self.time = []

        #self.add_on_shutdown_callback(self.on_shutdown)
        #self.add_on_shutdown_callback(self.on_shutdown)

        #self.subscription = self.create_subscription(
        #    Twist,
        #    incoming_twist_topic,
        #    self.cmd_vel_callback,
        #    10)
        self.msg_cmd_vel = Twist()
        self.follow_cmd = PoseStamped()
        self.received_first_cmd_vel = False
        timer_period = 0.2
        self.follow = self.create_subscription(PoseStamped,leader_robot_prefix+'/pose',self.follow_update_callback,100)
        self.height = self.create_subscription(PoseStamped,robot_prefix+'/pose',self.callback_height,100)
        #self.flag_sub = self.create_subscription(Bool,'/flag',self.flag_callback,10)
        self.timer = self.create_timer(timer_period, self.callback_timer)
        self.takeoff_client = self.create_client(Takeoff, robot_prefix + '/takeoff')
        self.publisher_hover = self.create_publisher(Hover, robot_prefix + '/cmd_hover', 10)
        self.land_client = self.create_client(Land, robot_prefix + '/land')
        self.cf_has_taken_off = False
        self.takeoff_client.wait_for_service()
        self.land_client.wait_for_service()
        self.flag_pos = True
        self.current_height = 0.0
        self.gain = 0.5

        self.get_logger().info(f"Follower initialized")

    ##def cmd_vel_callback(self, msg):
    ##    self.msg_cmd_vel = msg
    ##    # This is to handle the zero twist messages from teleop twist keyboard closing
    ##    # or else the crazyflie would constantly take off again.
    ##    msg_is_zero = msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0 and msg.linear.z == 0.0
    ##    if  msg_is_zero is False and self.received_first_cmd_vel is False and msg.linear.z >= 0.0:
        

    ##        self.received_first_cmd_vel = True
        
    #def flag_callback(self,msg):
    #    self.flag_pos = msg.data

    def callback_height(self,msg):
        self.z_pos.append(msg.pose.position.z)
        self.time.append(msg.header.stamp.sec)
        self.current_height = msg.pose.position.z

    def follow_update_callback(self,msg):
        self.follow_cmd=msg
        ##self.get_logger().info("Info Recorded")
        
    def callback_timer(self):
        #self.get_logger().info(f"{self.flag_pos}")
        msg = Hover()
        if(self.flag_pos is True and self.current_height>0.08):
            ##self.get_logger().info("Halo")
            self.get_logger().info("Holaaaaaaaaaaaaaaaaaaaaaaaaaaa")
            msg.vx = 0.0
            msg.vy = 0.0
            msg.yaw_rate = self.follow_cmd.pose.orientation.z
            msg.z_distance = self.follow_cmd.pose.position.z
            #msg.z_distance = 0.5
            self.publisher_hover.publish(msg)
        else:
            msg.vx = 0.0
            msg.vy = 0.0
            msg.yaw_rate = 0.0
            msg.z_distance = 0.1
            self.publisher_hover.publish(msg)

    
    def call_land_callback(self):
        req = Land.Request()
        req.height = 0.05
        req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
        self.land_client.call_async(req)
        self.get_logger.warn("Inside Land")
        time.sleep(2.0)        
        self.cf_has_taken_off = False
        self.received_first_cmd_vel = False
    
    def write_into_file(self):
        z_mat=np.array(self.z_pos)
        t_mat=np.array(self.time)
        scipy.io.savemat('/home/abd/exp_ws/ros_bag_files/cf_follower.mat', dict(t1=t_mat, z1=z_mat))
    def on_shutdown(self):
        self.get_logger().info('Shutting down...')
        self.write_into_file()


def main(args=None):
    rclpy.init(args=args)

    follower_node = FollowerNode()
    try:
        rclpy.spin(follower_node)
    #except KeyboardInterrupt:
    #    print("Ctrl+C pressed. Landing Crazyflie...")
    #    follower_node.get_logger().info('Keyboard Interrupt')
    #    follower_node.call_land_callback()
    finally:
        follower_node.on_shutdown()
        follower_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
