#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, Empty
from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from crazyflie_interfaces.msg import Hover
import scipy.io
import numpy as np
from cflib.utils.power_switch import PowerSwitch
from builtin_interfaces.msg import Duration
import time

class ControllerNode(Node):
    def __init__(self):
        super().__init__('position_controller')
        self.declare_parameter('hover_height', 0.05)
        self.declare_parameter('incoming_twist_topic', '/cmd_vel')
        ##################################################################
        self.declare_parameter('robot_prefix', '/cf3')
        self.declare_parameter('power_uri', 'radio://0/80/2M/E7E7E7E703')

        self.hover_height = self.get_parameter('hover_height').value
        robot_prefix = self.get_parameter('robot_prefix').value
        incoming_twist_topic = self.get_parameter('incoming_twist_topic').value
        power_uri = self.get_parameter('power_uri').value

        self.power_switch = PowerSwitch(power_uri)

        ##################################################################
        # --------------- POWER MANAGEMENT SERVICE CALLS --------------- #
        self.cli_power_down = self.create_client(Trigger, 'power_down')
        self.cli_power_up = self.create_client(Trigger, 'power_up')
        ##################################################################

        self.takeoff_client = self.create_client(Takeoff, robot_prefix + '/takeoff')
        self.land_client = self.create_client(Land, robot_prefix + '/land')
        self.notify_client = self.create_client(NotifySetpointsStop, robot_prefix + '/notify_setpoints_stop')
        self.emergency_client = self.create_client(Empty, 'all/emergency')

        self.x_pos = []
        self.y_pos = []
        self.z_pos = []
        self.des_x_pos = []
        self.des_y_pos = []
        self.des_z_pos = []
        self.time = []

        self.desired_x_position = 0.0
        self.desired_y_position = 0.0
        self.desired_z_position = 0.0
        self.received_first_cmd_vel = False
        self.cf_has_taken_off = False
        self.first_val = False
        self.init_x = 0.0
        self.init_y = 0.0
        self.init_z = 0.0

        self.subscription = self.create_subscription(
            Twist,
            incoming_twist_topic,
            self.cmd_vel_callback,
            10
        )
        self.publisher_setpoint = self.create_publisher(Twist, robot_prefix + '/cmd_pos', 10)
        self.height_subscription = self.create_subscription(
            PoseStamped,
            robot_prefix + '/pose',
            self.callback_height,
            100
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.takeoff_client.wait_for_service()
        self.land_client.wait_for_service()

        self.last_cmd_vel_time = time.time() #######################

        self.get_logger().info(f"Velocity Multiplexer set for {robot_prefix} with height {self.hover_height} m using the {incoming_twist_topic} topic")

    def cmd_vel_callback(self, msg):
        self.msg_cmd_vel = msg
        self.last_cmd_vel_time = time.time()
        self.desired_x_position = msg.linear.x
        self.desired_y_position = msg.linear.y
        self.desired_z_position = msg.linear.z
        self.received_first_cmd_vel = True
        self.get_logger().info("Entered cmd_vel_callback")
        self.cf_has_taken_off = True

    def position_to_velocity(self, msg):
        self.desired_x_position = msg.linear.x
        self.desired_y_position = msg.linear.y
        self.desired_z_position = msg.linear.z
        K = 1.2
        vx = -(self.x_position - self.desired_x_position) * K
        vy = -(self.y_position - self.desired_y_position) * K
        ret_msg = Hover()
        ret_msg.vx = vx
        ret_msg.vy = vy
        ret_msg.yaw_rate = 0.0
        ret_msg.z_distance = self.desired_z_position
        return ret_msg

    def callback_height(self, msg):
        if not self.first_val:
            self.init_x = msg.pose.position.x
            self.init_y = msg.pose.position.y
            self.init_z = msg.pose.position.z
            self.get_logger().info("Successfully Set the Origin")
            self.first_val = True

        self.x_position = msg.pose.position.x - self.init_x
        self.y_position = msg.pose.position.y - self.init_y
        self.z_position = msg.pose.position.z - self.init_z

        self.x_pos.append(self.x_position)
        self.y_pos.append(self.y_position)
        self.z_pos.append(self.z_position)
        self.des_x_pos.append(self.desired_x_position)
        self.des_y_pos.append(self.desired_y_position)
        self.des_z_pos.append(self.desired_z_position)
        self.time.append(msg.header.stamp.sec)

    def timer_callback(self):
        ######################################################################################
        # CAN COMMENT THIS OUT MAYBE
        if time.time() - self.last_cmd_vel_time > 5.0:  # 5 seconds of timeout
            self.get_logger().warn('No cmd_vel received for 5 seconds, initiating landing.')
            self.initiate_landing()
            return
        ######################################################################################

        if self.received_first_cmd_vel and self.cf_has_taken_off:
            if self.msg_cmd_vel.linear.z >= 0:
                msg = Twist()
                msg.linear.x = self.msg_cmd_vel.linear.x + self.init_x
                msg.linear.y = self.msg_cmd_vel.linear.y + self.init_y
                msg.linear.z = self.msg_cmd_vel.linear.z + self.init_z
                if self.publisher_setpoint.get_publisher_context() is not None:
                    self.publisher_setpoint.publish(msg)
                    self.get_logger().info("x: " + str(self.x_position) + " y: " + str(self.y_position) + " z: " + str(self.z_position))
                else:
                    self.get_logger().error("Publisher context is invalid.")
            else:
                # req = NotifySetpointsStop.Request()
                # self.notify_client.call_async(req)
                # req = Land.Request()
                # req.height = 0.1
                # req.duration = Duration(sec=2, nanosec=0)
                # self.land_client.call_async(req)
                # self.get_logger().info("Landing sequence initiated -------------")
                # self.create_timer(2.0, self.handle_landing)
                self.get_logger().info("Z<0 :: Landing sequence initiated ")
                self.initiate_landing()
    
    def initiate_takeoff(self):
        takeoff_req = Takeoff.Request()
        takeoff_req.height = self.hover_height
        takeoff_req.duration = Duration(sec=2, nanosec=0)
        self.takeoff_client.call_async(takeoff_req)
        self.get_logger().info("Takeoff sequence initiated.")
        self.cf_has_taken_off = True

    def initiate_landing(self):
        # Notify Crazyflie to stop setpoints and land the drone
        notify_stop_req = NotifySetpointsStop.Request()
        self.notify_client.call_async(notify_stop_req)

        land_req = Land.Request()
        land_req.height = 0.1
        land_req.duration = Duration(sec=2, nanosec=0)
        self.land_client.call_async(land_req)

        self.get_logger().info("Landing sequence initiated.")
        # Ensure only one timer for handling landing
        if self.landing_timer is None:
            self.landing_timer = self.create_timer(3.0, self.handle_landing)  # Call `handle_landing` after 3 seconds to ensure landing is completed

    def handle_landing(self):
        self.cf_has_taken_off = False
        self.received_first_cmd_vel = False
        self.get_logger().info("!!!!!!!! Landing completed !!!!!!!!")
        # Clean up the timer
        if self.landing_timer:
            self.landing_timer.destroy()
            self.landing_timer = None

    def write_into_file(self):
        x_mat = np.array(self.x_pos)
        y_mat = np.array(self.y_pos)
        z_mat = np.array(self.z_pos)
        xd_mat = np.array(self.des_x_pos)
        yd_mat = np.array(self.des_y_pos)
        zd_mat = np.array(self.des_z_pos)
        t_mat = np.array(self.time)
        scipy.io.savemat('/home/anuraag/Downloads/rosbag_cf/logging.mat', dict(t1=t_mat, x1=x_mat, y1=y_mat, z=z_mat, xd=xd_mat, yd=yd_mat, zd=zd_mat))
        self.get_logger().info("Data written to file @ /home/anuraag/Downloads/rosbag_cf/logging.mat")

    def on_shutdown(self):
        self.get_logger().info('Shutting down...')
        self.powerdown()
        self.write_into_file()
        self.get_logger().info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!! File written !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")        

#############################################################################################################
    # ------------------------- POWER MANAGEMENT ------------------------- #
    def powerdown(self):
        if not self.cli_power_down.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Power down service not available')
            return
        
        request = Trigger.Request()
        future = self.cli_power_down.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response.success:
            self.get_logger().info('Successfully powered down')
        else:
            self.get_logger().error(f'Failed to power down: {response.message}')

    def powerup(self):
        if not self.cli_power_up.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Power up service not available')
            return
        
        request = Trigger.Request()
        future = self.cli_power_up.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response.success:
            self.get_logger().info('Successfully powered up')
        else:
            self.get_logger().error(f'Failed to power up: {response.message}\n!!!! :((((((((( REBOOT !!!!!')
#############################################################################################################
#############################################################################################################

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        # Uncomment this line if you want to handle emergency stop on interruption
        request = Empty.Request()
        controller_node.emergency_client.call_async(request)
        print("Interrupt from Keyboard Exception") 
    finally:
        print("Final Interruption")
        controller_node.on_shutdown()

    # Introduce a short delay to ensure shutdown processes complete
    time.sleep(1.0)
    
    controller_node.powerup() 
    print("((((((((((((((((((((((((((((((((((((((((( POWERING UP )))))))))))))))))))))))))))))))))))))))))")
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()