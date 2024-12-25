#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from crazyflie_interfaces.msg import Hover
import time
from geometry_msgs.msg import Twist,PoseStamped
import numpy as np

class Trajectory(Node):
    def __init__(self):
        super().__init__('trajectory')
        self.publisher_command = self.create_publisher(Twist,'/cmd_vel', 10)
        self.position = self.create_subscription(PoseStamped,'cf1/pose',self.callback_position,100)
        self.timer = self.create_timer(0.5,self.timer_callback)
        self.threshold_publisher = self.create_publisher(Twist,'/threshold',10)
        self.x = [0.0,0.3,0.3,0.0,0.0,0.0]
        self.y = [0.0,0.0,0.3,0.3,0.0,0.0]
        self.z = [0.2,0.3,0.4,0.3,0.4,0.2]
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.index = 0
        self.i = True
        self.threshold = 0.01
        self.x_pos=0.0
        self.y_pos=0.0
        self.z_pos=0.0
    
    def publish_message(self):
        msg=Twist()
        #time.sleep(3.0)
        self.x_pos = self.x[self.index]
        self.y_pos = self.y[self.index]
        self.z_pos = self.z[self.index]
        msg.linear.x = self.x_pos
        msg.linear.y = self.y_pos
        msg.linear.z = self.z_pos

        self.publisher_command.publish(msg)
        self.get_logger().info("........................................................................................Publishing Now..........................................................................................................................................")
        self.index=self.index+1
    def callback_position(self,msg):
        if self.i is True:
            self.init_x = msg.pose.position.x
            self.init_y = msg.pose.position.y
            self.init_z = msg.pose.position.z
            self.i=False
            self.get_logger().info("...............................................................................................................Set origin.....................................................")
        
        self.current_x = msg.pose.position.x-self.init_x
        self.current_y = msg.pose.position.y-self.init_y
        self.current_z = msg.pose.position.z-self.init_z


    def has_reached_point(self,current_pos,desired_pos):
        distance_to_target = self.calculate_distance(current_pos,desired_pos)
        return distance_to_target <= self.threshold
    
    def calculate_distance(self,current_pos,desired_pos):
       return np.linalg.norm(np.array(current_pos) - np.array(desired_pos))

    def timer_callback(self):
        current_pos = (self.current_x,self.current_y,self.current_z)
        desired_pos = (self.x_pos,self.y_pos,self.z_pos)
        msg=Twist()
        msg.linear.x = self.calculate_distance(current_pos,desired_pos)
        msg.angular.x = self.x_pos-self.current_x
        msg.angular.y = self.y_pos-self.current_y
        msg.angular.z = self.z_pos-self.current_z
        self.threshold_publisher.publish(msg)
        if self.has_reached_point(current_pos,desired_pos):
            print(self.has_reached_point(current_pos,desired_pos))
            self.publish_message()
        if(self.index==len(self.x)):
            self.index=0
            

def main(args=None):
    rclpy.init(args=args)

    trajectory_node = Trajectory()
    time.sleep(5.0)
    trajectory_node.publish_message()
    rclpy.spin(trajectory_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()