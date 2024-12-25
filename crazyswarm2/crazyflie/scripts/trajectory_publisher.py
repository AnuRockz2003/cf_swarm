#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import Bool
import time


class TrajectoryPublisherNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("trajectory_publisher") # MODIFY NAME
        self.initial_point = 0.10
        self.ascent = True
        self.flag = False 
        self.gain = 0.5
        timer_period = 0.5     ##incoming_twist_topic  = self.get_parameter('incoming_twist_topic').value
        self.publisher_hover = self.create_publisher(Twist,'/cmd_vel',10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.flag_to_start = self.create_subscription(Bool,'/flag',self.flag_callback,10)
        self.height_cf1 = self.create_subscription(PoseStamped,'/cf1/pose',self.callback_height_cf1,100)
        self.height_cf2 = self.create_subscription(PoseStamped,'/cf2/pose',self.callback_height_cf2,100)
        self.current_height_cf1 = 0.0
        self.current_height_cf2 = 0.0

    def flag_callback(self,msg):
        self.flag = msg.data
        
    def callback_height_cf1(self,msg):
        self.current_height_cf1 = msg.pose.position.z
    def callback_height_cf2(self,msg):
        self.current_height_cf2 = msg.pose.position.z
    def timer_callback(self):
        ##self.get_logger().info(f"{self.flag}")
        msg = Twist()
        if(self.flag is True and self.current_height_cf1>0.08):
            self.get_logger().info(f"{self.current_height_cf1}")
            self.get_logger().info(f"{self.current_height_cf2}")
            self.get_logger().error("Entered Time callback")
            if(self.ascent is True):  
                msg.linear.x = 0.00
                msg.linear.y = 0.00
                msg.linear.z = self.initial_point
                msg.angular.x = 0.00
                msg.angular.y = 0.00
                msg.angular.z = 0.00
                self.publisher_hover.publish(msg)
                if(self.current_height_cf2>self.current_height_cf1-0.01):
                    self.initial_point=self.initial_point+0.01
                    self.get_logger().info("Ascent Message published")
                if(self.initial_point >= 0.3):
                    self.ascent=False
                
            elif(self.ascent is False):
                msg.linear.x = 0.00
                msg.linear.y = 0.00
                msg.linear.z = self.initial_point
                msg.angular.x = 0.00
                msg.angular.y = 0.00
                msg.angular.z = 0.00    
                self.publisher_hover.publish(msg)
                if(self.current_height_cf2>self.current_height_cf1-0.01):
                    self.initial_point=self.initial_point-0.01
                    self.get_logger().warn("Descent Message published")     
        else:
            msg.linear.x = 0.00
            msg.linear.y = 0.00
            msg.linear.z = 0.1
            msg.angular.x = 0.00
            msg.angular.y = 0.00
            msg.angular.z = 0.00
            self.publisher_hover.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisherNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()