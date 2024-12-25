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

from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import Bool
from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from crazyflie_interfaces.msg import Hover
import time
import scipy.io
import numpy as np

class LeaderNode(Node):
    def __init__(self):
        super().__init__('velocity_controller_test')
        self.declare_parameter('hover_height', 0.05)
        self.declare_parameter('robot_prefix', '/cf')
        self.declare_parameter('incoming_twist_topic', '/cmd_vel')

        self.hover_height  = self.get_parameter('hover_height').value
        robot_prefix  = self.get_parameter('robot_prefix').value
        incoming_twist_topic  = self.get_parameter('incoming_twist_topic').value

        self.z_pos = []
        self.x_pos = []
        self.y_pos = []

        self.des_x_pos=[]
        self.des_y_pos=[]
        self.des_z_pos=[]

        self.time = []

        self.desired_x_position=0.0
        self.desired_y_position=0.0
        self.desired_z_position=0.0
        
        self.subscription = self.create_subscription(
            Twist,
            incoming_twist_topic,
            self.cmd_vel_callback,
            10)
        self.msg_cmd_vel = Twist()
        self.received_first_cmd_vel = False
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.takeoff_client = self.create_client(Takeoff, robot_prefix + '/takeoff')
        self.publisher_hover = self.create_publisher(Hover, robot_prefix + '/cmd_hover', 10)
        self.land_client = self.create_client(Land, robot_prefix + '/land')
        self.height = self.create_subscription(PoseStamped,robot_prefix+'/pose',self.callback_height,100)
        self.notify_client = self.create_client(NotifySetpointsStop, robot_prefix + '/notify_setpoints_stop')
        ##self.flag = self.create_subscription(Bool,'/flag',self.callback_flag,100)
        self.cf_has_taken_off = True
        self.first_val = False
        self.init_x = 0.0
        self.init_y = 0.0
        self.init_z = 0.0
        
        ## Publisher to publish the current position of leader crazyflie to the follower crazyflie

        self.takeoff_client.wait_for_service()
        self.land_client.wait_for_service()
        #self.current_height = float

        self.get_logger().info(f"Velocity Multiplexer set for {robot_prefix}"+
                               f" with height {self.hover_height} m using the {incoming_twist_topic} topic")

    def cmd_vel_callback(self, msg):
        self.msg_cmd_vel = msg
        # This is to handle the zero twist messages from teleop twist keyboard closing
        # or else the crazyflie would constantly take off again.
        self.received_first_cmd_vel = True
        self.get_logger().info("Entered vel call")

        self.cf_has_taken_off = True
    
    def position_to_velocity(self,msg):
        self.desired_x_position = msg.linear.x
        self.desired_y_position = msg.linear.y
        self.desired_z_position = msg.linear.z

        K = 1.2
        ## Now that we have got the x,y,z coordinates from the /cf1/cmd_vel topic we need to convert it to velocity control inputs before publishing it to the 
        ## Hover message

        vx = -(self.x_position-self.desired_x_position)*K
        vy = -(self.y_position-self.desired_y_position)*K

        ret_msg = Hover()
        
        ret_msg.vx = vx
        ret_msg.vy = vy
        ret_msg.yaw_rate = 0.0
        ret_msg.z_distance = self.desired_z_position

        return ret_msg
    
    def double_filter(msg):
        return 0


    
    def callback_height(self,msg):

    ## Wherever we place the Crazyflie it will take that point as the origin ##########################################
        if(self.first_val is False):
            self.init_x = msg.pose.position.x
            self.init_y = msg.pose.position.y
            self.init_z = msg.pose.position.z
            self.get_logger().info("............................Successfully Set the Origin .....................................................")
            self.first_val = True
    
    ##### Setting the position of the crazyflie with respect to the set ORIGIN ##########################################3
    
        self.x_position = msg.pose.position.x-self.init_x
        self.y_position = msg.pose.position.y-self.init_y
        self.z_position = msg.pose.position.z-self.init_z

    
    #### The position of the crazyflie is logged everytime the topic cf1/pose publishes message ##############################################

        self.z_pos.append(self.z_position)
        self.x_pos.append(self.x_position)
        self.y_pos.append(self.y_position)

        self.des_x_pos.append(self.desired_x_position)
        self.des_y_pos.append(self.desired_y_position)
        self.des_z_pos.append(self.desired_z_position)

        self.time.append(msg.header.stamp.sec)
    
    ###########################################################################################################################################

    def timer_callback(self):
        ##self.get_logger().info("Entered Timer call")
        #if self.received_first_cmd_vel and self.cf_has_taken_off is False:
        #    req = Takeoff.Request()
        #    req.height = self.hover_height
        #    req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
        #    self.takeoff_client.call_async(req)
        #    self.cf_has_taken_off = True
        #    time.sleep(2.0)        
        #    self.get_logger().info("Takeoff request Initiated")
        if self.received_first_cmd_vel and self.cf_has_taken_off:
            if self.msg_cmd_vel.linear.z >= 0:
                msg = Hover()
                msg = self.position_to_velocity(self.msg_cmd_vel)
                self.get_logger().info("Hover request Initiated")
                self.publisher_hover.publish(msg)
                self.get_logger().info("x:"+str(self.x_position)+" y: "+str(self.y_position)+" z: "+str(self.z_position))
            else:
                req = NotifySetpointsStop.Request()
                self.notify_client.call_async(req)
                req = Land.Request()
                req.height = 0.1
                req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
                self.land_client.call_async(req)
                time.sleep(2.0)        
                self.cf_has_taken_off = False
                self.received_first_cmd_vel = False
    def land_callback(self):
        req = Land.Request()
        req.height = 0.05
        req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
        self.land_client.call_async(req)
        time.sleep(2.0)        
        self.cf_has_taken_off = False
        self.received_first_cmd_vel = False
    
    def write_into_file(self):
        z_mat=np.array(self.z_pos)
        y_mat=np.array(self.y_pos)
        x_mat=np.array(self.x_pos)

        zd_mat=np.array(self.des_z_pos)
        yd_mat=np.array(self.des_y_pos)
        xd_mat=np.array(self.des_x_pos)

        t_mat=np.array(self.time)
        
        scipy.io.savemat('/home/abd/exp_ws/ros_bag_files/cf_leader.mat', dict(t1=t_mat, x1=x_mat,y1=y_mat,z=z_mat,xd=xd_mat,yd=yd_mat,zd=zd_mat))

    def on_shutdown(self):
        self.get_logger().info('Shutting down...')
        self.write_into_file()



def main(args=None):
    rclpy.init(args=args)

    leader_node = LeaderNode()
    try:
        rclpy.spin(leader_node)
    #except KeyboardInterrupt:
    #    leader_node.land_callback()
    finally:
        leader_node.on_shutdown()
        leader_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
