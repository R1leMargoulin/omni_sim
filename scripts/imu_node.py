#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

import numpy as np
#from filterpy.kalman import KalmanFilter
from witmotion import IMU

class ImuNode(Node):

  def __init__(self):
    super().__init__('imu_node')
    #Create the imu
    self.imu = IMU(path="/dev/ttyS0")
    # Create the Kalman filter
    #self.kf = KalmanFilter(dim_x=2, dim_z=1)

    # Create the IMU publisher
    self.imu_pub = self.create_publisher(Imu, '/imu', 10)
    

    
    timer_period = 0.01
    self.timer = self.create_timer(timer_period, self.timer_callback)

    

  def timer_callback(self):
    imu_msg = Imu()

    # Update the Kalman filter with the new measurements
    #self.kf.update(self.imu.get_quaternion(), self.imu.get_angular_velocity(), self.imu.get_acceleration())

    # Get the estimated orientation, angular velocity, and linear acceleration from the Kalman filter
    imu_orientation = self.imu.get_angle()# self.kf.get_orientation()
    imu_angular_velocity = self.imu.get_angular_velocity()# self.kf.get_angular_velocity()
    imu_linear_acceleration = self.imu.get_acceleration()# self.kf.get_linear_acceleration()

    if(imu_angular_velocity != None):
      angular_velocity = Vector3()
      angular_velocity.x = imu_angular_velocity[0]
      angular_velocity.y = imu_angular_velocity[1]
      angular_velocity.z = imu_angular_velocity[2]

      imu_msg.angular_velocity = angular_velocity

    if(imu_linear_acceleration != None):
      linear_acceleration = Vector3()
      linear_acceleration.x = imu_linear_acceleration[0]
      linear_acceleration.y = imu_linear_acceleration[1]
      linear_acceleration.z = imu_linear_acceleration[2]

      imu_msg.linear_acceleration = linear_acceleration

    if(imu_orientation != None):
      quaternion = self.euler_to_quaternion(imu_orientation[0], imu_orientation[1], imu_orientation[2])
      imu_msg.orientation = quaternion

    # Publish the estimated IMU data
    self.imu_pub.publish(imu_msg)

  
  def euler_to_quaternion(self, roll, pitch, yaw):
    q = Quaternion()
    q.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    q.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    q.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    q.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return q



def main():
  rclpy.init()

  imu_node = ImuNode()


  try:
    # Spin until ROS is terminated
    rclpy.spin(imu_node)
  except:
    imu_node.imu.close()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
