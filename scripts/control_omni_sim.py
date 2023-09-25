#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

import time
import numpy as np
import serial as s

#node class
class ulOmnibotControl(Node):

    def __init__(self):
        super().__init__('omni_sim_control')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('LISTEN_TOPIC_NAME', rclpy.Parameter.Type.STRING),
                ('PUBLISH_TOPIC_NAME', rclpy.Parameter.Type.STRING),
                ('MAX_SPEED_X', rclpy.Parameter.Type.INTEGER),
                ('MAX_SPEED_Y', rclpy.Parameter.Type.INTEGER),
                ('MAX_SPEED_W', rclpy.Parameter.Type.INTEGER),
                ('Wheel_center_distance', rclpy.Parameter.Type.DOUBLE),
                ('Wheel_radius', rclpy.Parameter.Type.DOUBLE)
            ])
        #Variables
        LISTEN_TOPIC_NAME = self.get_parameter('LISTEN_TOPIC_NAME').get_parameter_value().string_value
        PUBLISH_TOPIC_NAME = self.get_parameter('PUBLISH_TOPIC_NAME').get_parameter_value().string_value

        self.MAX_SPEED_X = self.get_parameter('MAX_SPEED_X').get_parameter_value().integer_value
        self.MAX_SPEED_Y = self.get_parameter('MAX_SPEED_Y').get_parameter_value().integer_value
        self.MAX_SPEED_W = self.get_parameter('MAX_SPEED_W').get_parameter_value().integer_value



        self.subscription = self.create_subscription(Twist, LISTEN_TOPIC_NAME, self.cmd_vel_callback, 10)
        self.publisher_ = self.create_publisher(Float64MultiArray, PUBLISH_TOPIC_NAME, 10)
        self.subscription  # prevent unused variable warning

        self.wheel_vel = [0,0,0,0]

        self.vel_x = 0
        self.vel_y = 0
        self.vel_w = 0
        
        self.L = self.get_parameter('Wheel_center_distance').get_parameter_value().double_value
        self.Rw = self.get_parameter('Wheel_center_distance').get_parameter_value().double_value

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):

        self.wheel_vel[0] = (-self.vel_x*np.sin(0       ) + self.vel_y*np.cos(0       ) + self.L*self.vel_w)/self.Rw
        self.wheel_vel[1] = (-self.vel_x*np.sin(np.pi/2) + self.vel_y*np.cos(np.pi/2) + self.L*self.vel_w)/self.Rw
        self.wheel_vel[2] = (-self.vel_x*np.sin(-np.pi)   + self.vel_y*np.cos(-np.pi)   + self.L*self.vel_w)/self.Rw
        self.wheel_vel[3] = (-self.vel_x*np.sin(-np.pi/2) + self.vel_y*np.cos(-np.pi/2) + self.L*self.vel_w)/self.Rw

        array_forPublish = Float64MultiArray(data=self.wheel_vel)    
        #rclpy.logging._root_logger.info(f"wheel vel : {self.wheel_vel}")
        self.publisher_.publish(array_forPublish)

    def cmd_vel_callback(self, msg):

        #command speed verifications
        if msg.linear.x > self.MAX_SPEED_X:
            self.vel_x = self.MAX_SPEED_X
            self.get_logger().info("LINEAR X OUT OF LIMIT")
        elif msg.linear.x < -self.MAX_SPEED_X:
            self.vel_x = -self.MAX_SPEED_X
            self.get_logger().info("LINEAR X OUT OF LIMIT")
        else:
            self.vel_x = msg.linear.x

        if msg.linear.y > self.MAX_SPEED_Y:
            self.vel_y = self.MAX_SPEED_Y
            self.get_logger().info("LINEAR Y OUT OF LIMIT")
        elif msg.linear.y < -self.MAX_SPEED_Y:
            self.vel_y = -self.MAX_SPEED_Y
            self.get_logger().info("LINEAR Y OUT OF LIMIT")
        else:
            self.vel_y = msg.linear.y

        if msg.angular.z > self.MAX_SPEED_W:
            self.vel_w = self.MAX_SPEED_W
            self.get_logger().info("ANGULAR OUT OF LIMIT")
        elif msg.angular.z < -self.MAX_SPEED_W:
            self.vel_w = -self.MAX_SPEED_W
            self.get_logger().info("ANGULAR OUT OF LIMIT")
        else:
            self.vel_w = msg.angular.z

        
def main(args=None):
    rclpy.init(args=args)

    omni_sim_control = ulOmnibotControl()

    rclpy.spin(omni_sim_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    omni_sim_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()