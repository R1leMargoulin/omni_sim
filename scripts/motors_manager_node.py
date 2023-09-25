#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

import time
import numpy as np
import serial as s

#node class
class MotorsControl(Node):

    def __init__(self):
        super().__init__('motors_control')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('DRIVER_A_ADDRESS', rclpy.Parameter.Type.INTEGER),
                ('DRIVER_B_ADDRESS', rclpy.Parameter.Type.INTEGER),
                ('MOT1_COMMAND', rclpy.Parameter.Type.INTEGER),
                ('MOT2_COMMAND', rclpy.Parameter.Type.INTEGER),
                ('ORDER_TOPIC_NAME', rclpy.Parameter.Type.STRING),
                ('MAX_SPEED_MOTOR', rclpy.Parameter.Type.INTEGER),
                ('MIN_SPEED_MOTOR', rclpy.Parameter.Type.INTEGER),
                ('motor_drivers_serial_port', rclpy.Parameter.Type.STRING)
            ])
        #Variables
        
        self.DRIVER_A_ADDRESS = self.get_parameter('DRIVER_A_ADDRESS').get_parameter_value().integer_value
        self.DRIVER_B_ADDRESS = self.get_parameter('DRIVER_B_ADDRESS').get_parameter_value().integer_value
        self.MOT1_COMMAND = self.get_parameter('MOT1_COMMAND').get_parameter_value().integer_value
        self.MOT2_COMMAND = self.get_parameter('MOT2_COMMAND').get_parameter_value().integer_value
        self.MAX_SPEED_MOTOR = self.get_parameter('MAX_SPEED_MOTOR').get_parameter_value().integer_value
        self.MIN_SPEED_MOTOR = self.get_parameter('MIN_SPEED_MOTOR').get_parameter_value().integer_value

        ORDER_TOPIC_NAME = self.get_parameter('ORDER_TOPIC_NAME').get_parameter_value().string_value
        sPort = self.get_parameter('motor_drivers_serial_port').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(Float64MultiArray, ORDER_TOPIC_NAME, self.order_callback, 10)
        self.subscription  # prevent unused variable warning
        self.serial = s.Serial(sPort) 



    def order_callback(self, msg):
        # speeds are organized such as: [2B, 1B, 2A, 1A]
        self.move(msg.data)


    # 2B/1      1A/4
    # 
    # 
    # 1B/2      2A/3

    def move(self, speeds):

        # speeds are organized such as: [2B, 1B, 2A, 1A]

        #Verify motor speeds
        if speeds[0] > self.MAX_SPEED_MOTOR :
            v2B = self.MAX_SPEED_MOTOR
        elif speeds[0] < self.MIN_SPEED_MOTOR :
            v2B = self.MIN_SPEED_MOTOR
        else:
            v2B = speeds[0]

        if speeds[1] > self.MAX_SPEED_MOTOR :
            v1B = self.MAX_SPEED_MOTOR
        elif speeds[1] < self.MIN_SPEED_MOTOR :
            v1B = self.MIN_SPEED_MOTOR
        else:
            v1B = speeds[1]

        if speeds[2] > self.MAX_SPEED_MOTOR :
            v2A = self.MAX_SPEED_MOTOR
        elif speeds[2] < self.MIN_SPEED_MOTOR :
            v2A = self.MIN_SPEED_MOTOR
        else:
            v2A = speeds[2]

        if speeds[3] > self.MAX_SPEED_MOTOR :
            v1A = self.MAX_SPEED_MOTOR
        elif speeds[3] < self.MIN_SPEED_MOTOR :
            v1A = self.MIN_SPEED_MOTOR
        else:
            v1A = speeds[3]
        
        
        
        #changing speed in good format for the driver's protocol
        v1A_data = int(np.interp(v1A,[-self.MAX_SPEED_MOTOR, self.MAX_SPEED_MOTOR], [0,127]))
        v1B_data = int(np.interp(v1B,[-self.MAX_SPEED_MOTOR, self.MAX_SPEED_MOTOR], [0,127]))
        v2A_data = int(np.interp(v2A,[-self.MAX_SPEED_MOTOR, self.MAX_SPEED_MOTOR], [0,127]))
        v2B_data = int(np.interp(v2B,[-self.MAX_SPEED_MOTOR, self.MAX_SPEED_MOTOR], [0,127]))

        #RPM or m/s ?????????????????????????????????????????????????????????

        #building message
        mot1A_message = self.get_motor_message(self.DRIVER_A_ADDRESS, self.MOT1_COMMAND, v1A_data)
        mot1B_message = self.get_motor_message(self.DRIVER_B_ADDRESS, self.MOT1_COMMAND, v1B_data)
        mot2A_message = self.get_motor_message(self.DRIVER_A_ADDRESS, self.MOT2_COMMAND, v2A_data)
        mot2B_message = self.get_motor_message(self.DRIVER_B_ADDRESS, self.MOT2_COMMAND, v2B_data)

        #sending the message
        self.serial.write(mot1A_message)
        self.serial.write(mot1B_message)
        self.serial.write(mot2A_message)
        self.serial.write(mot2B_message)


    def get_motor_message(self, address: int, command: int, data: int) -> bytes:
        """
        Create message given the address, command, and data bytes.
        The checksum is calculated from these values.
        """
        return bytes([address, command, data, self.get_checksum(address, command, data)])

    def get_checksum(self, address: int, command: int, data: int) -> bytes:
        """
        Get checksum for message given address, command, and data
        """
        return (address + command + data) & 127

        
def main(args=None):
    rclpy.init(args=args)

    motors = MotorsControl()

    rclpy.spin(motors)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()