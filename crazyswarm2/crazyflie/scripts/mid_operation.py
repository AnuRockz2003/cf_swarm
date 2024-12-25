#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import Bool
from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from crazyflie_interfaces.msg import Hover
import time



class MidOperationNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("mid_operation") # MODIFY NAME
        self.declare_parameter('hover_height', 0.05)
        self.declare_parameter('robot_prefix', '/cf')
        self.declare_parameter('incoming_twist_topic', '/cmd_vel')
        

        self.hover_height  = self.get_parameter('hover_height').value
        robot_prefix  = self.get_parameter('robot_prefix').value
        incoming_twist_topic  = self.get_parameter('incoming_twist_topic').value

        self.position_data = self.create_subscription(PoseStamped,robot_prefix+'/pose',self.callback_position,100)
        
        #self.subscription = self.create_subscription(
        #    Twist,
        #    incoming_twist_topic,
        #    self.cmd_vel_callback,
        #    10)
        self.msg_cmd_vel = Twist()
        self.received_first_cmd_vel = False
        #timer_period = 0.1
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.takeoff_client = self.create_client(Takeoff, robot_prefix + '/takeoff')
        self.publisher_hover = self.create_publisher(Hover, robot_prefix + '/cmd_hover', 10)
        self.land_client = self.create_client(Land, robot_prefix + '/land')
        self.notify_client = self.create_client(NotifySetpointsStop, robot_prefix + '/notify_setpoints_stop')
        self.cf_has_taken_off = False
        self.timer = self.create_timer(0.1,self.timer_callback)
        self.finish_sub= self.create_subscription(Bool,'/flag',self.finish_callback,10)
        self.takeoff_client.wait_for_service()
        self.land_client.wait_for_service()
        self.flak = Bool
        self.current_height = 0.0
        self.hover_heightt = 0.3

        self.get_logger().info(f"Velocity Multiplexer set for {robot_prefix}"+
                               f" with height {self.hover_height} m using the {incoming_twist_topic} topic")
        ##self.req = Takeoff.Request()
        ##self.req.height = self.hover_height
        ##self.req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
        ##self.takeoff_client.call_async(self.req)
        ##self.cf_has_taken_off = True
        ##time.sleep(2.0) 
        ##self.msg = Hover()
        ##self.msg.vx = 0.0
        ##self.msg.vy = 0.0
        ##self.msg.yaw_rate = 0.0
        ##self.msg.z_distance = self.hover_height
        ##self.publisher_hover.publish(self.msg)
        ##self.msg = Bool()
        ##self.msg.data = True
        ##self.finish_pub.publish(self.msg)
        ##time.sleep(2.0)

    #def timer_callback(self):
    #    self.get_logger().info("Published conformation")
    def finish_callback(self,msg):
        self.flak.data = msg.data

    def callback_position(self,msg):
        self.current_height = msg.pose.position.z

    def timer_callback(self):
        self.hvr_msg=Hover()
        if(self.flak.data is True and self.current_height>0.08):
        #if(self.flak.data is True):
            self.hvr_msg.vx = 0.0
            self.hvr_msg.vy = 0.0
            self.hvr_msg.yaw_rate = 0.0
            self.hvr_msg.z_distance = 0.3
            self.publisher_hover.publish(self.hvr_msg)
            self.get_logger().info("Heloooooooooooooo")
        else:
            self.hvr_msg.vx = 0.0
            self.hvr_msg.vy = 0.0
            self.hvr_msg.yaw_rate = 0.0
            self.hvr_msg.z_distance = 0.1
            self.publisher_hover.publish(self.hvr_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MidOperationNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()