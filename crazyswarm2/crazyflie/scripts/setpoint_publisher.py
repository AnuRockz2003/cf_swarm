#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from crazyflie_interfaces.msg import Hover
import time

timer_period = 4.0
class SetpointPublisher(Node):
    def __init__(self):
        super().__init__("setpoint_publisher")
        self.sequence = [
    (0.0, 0.0, 0.3, 0),
    (0.0, 0.0, 0.2, 0),
    (0.2, 0.1, 0.3, 0),
    (0.2, 0.3, 0.2, 0),
    (-0.2, 0.2, 0.2, 0),
    (-0.2, 0.0, 0.3, 0),
    (0.0, 0.0, 0.4, 0),
    (0.0, 0.0, 0.1, 0),
                         ]
        self.index=0
        self.publisher_setpoint = self.create_publisher(Twist,'/cmd_vel',10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        print("HELOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")


    def timer_callback(self):
        if(self.index<len(self.sequence)):
            msg = Twist()
            msg.linear.x = self.sequence[self.index][0]
            msg.linear.y = self.sequence[self.index][1]
            msg.linear.z = self.sequence[self.index][2]
            self.publisher_setpoint.publish(msg)
            self.index=self.index+1
            print(self.index)
        else:
            msg=Twist()
            msg.linear.x=0.0
            msg.linear.y=0.0
            msg.linear.z=0.0
            self.publisher_setpoint.publish(msg)
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node=SetpointPublisher()
    try:
        rclpy.spin(node)
    except SystemExit:
        
        rclpy.logging.get_logger("Quitting").info('Done')
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':

    main()



            


