import rclpy
from rclpy.node import Node

import time
from math import modf

from std_msgs.msg import Header
from custom_interfaces.msg import Controllermsg, Attitude, Imu

import sys
sys.path.append('/home/volk/Desktop/DJI_Robomaster/env/ros_ws/src/robomaster_s1/robomaster_s1')
import ps5controller as controller
from robomaster import robot

s1 = robot.Robot()
s1.initialize(conn_type='rndis')
s1.set_robot_mode(mode=robot.CHASSIS_LEAD)
s1_led = s1.led
s1_chassis= s1.chassis
s1_gimbal = s1.gimbal  

class robotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.subscription = self.create_subscription(Controllermsg, 'controller',self.chassis_control, 10)
        self.subscription

        s1_led.set_led(r=128,g=0,b=128) 

        #attitude publisher
        s1_chassis.sub_attitude(freq=50, callback=self.publish_attitude)

        self.attitude_msg = Attitude()
        self.attitude_msg.header = Header()
        self.attitude_msg.header.frame_id = 'base_link'
        self.attitude_msg.yaw = 0.0 # Yaw axis attitude angle
        self.attitude_msg.pitch = 0.0 # Pitch axis attitude angle
        self.attitude_msg.roll = 0.0 # Roll axis attitude angle
        self.attitude_pub = self.create_publisher(Attitude, 'attitude', 10)

        #imu publisher
        s1_chassis.sub_imu(freq=50, callback=self.publish_imu)

        self.imu_msg = Imu()
        self.imu_msg.header = Header()
        self.imu_msg.header.frame_id = 'base_link'
        self.imu_msg.acc_x = 0.0 # x-axis acceleration
        self.imu_msg.acc_y = 0.0 # y-axis acceleration
        self.imu_msg.acc_z = 0.0 # z-axis acceleration
        self.imu_msg.gyro_x = 0.0 # x-axis angular velocity
        self.imu_msg.gyro_y = 0.0 # y-axis angular velocity
        self.imu_msg.gyro_z = 0.0 # z-axis angular velocity
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)

        #camera publisher

    def chassis_control(self, msg):
        if msg.button == 'triangle' and msg.button_direction == 'down':
            print('Shutting down...')
            s1_led.set_led(r=255,g=0,b=0)
            s1.close()

        if msg.l2 != 0 and msg.r2 == 0:
            s1_chassis.drive_speed(x=msg.left_joystick[1],y=msg.left_joystick[0],z=msg.l2)
        elif msg.r2 != 0 and msg.l2 == 0:
            s1_chassis.drive_speed(x=msg.left_joystick[1],y=msg.left_joystick[0],z=msg.r2)
        else:
            s1_chassis.drive_speed(x=msg.left_joystick[1],y=msg.left_joystick[0],z=0)
        
        s1_gimbal.drive_speed(-msg.right_joystick[1],msg.right_joystick[0])

    def publish_attitude(self, data): #callback function
        self.attitude_msg.header.stamp = self.get_clock().now().to_msg()

        yaw, pitch, roll = data
        self.attitude_msg.yaw = yaw
        self.attitude_msg.pitch = pitch
        self.attitude_msg.roll = roll

        self.attitude_pub.publish(self.attitude_msg)

    def publish_imu(self, data): #callback function
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()

        ax, ay, az, wx, wy, wz = data
        self.imu_msg.acc_x = ax
        self.imu_msg.acc_y = ay
        self.imu_msg.acc_z = az
        self.imu_msg.gyro_x = wx
        self.imu_msg.gyro_y = wy
        self.imu_msg.gyro_z = wz

        self.imu_pub.publish(self.imu_msg)
    


def main(args=None):
    rclpy.init(args=args)

    robot_ros2 = robotNode()
    rclpy.spin(robot_ros2)

    # # Destroy the node explicitly
    robot_ros2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()