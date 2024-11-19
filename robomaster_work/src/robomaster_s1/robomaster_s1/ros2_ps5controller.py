import rclpy
from rclpy.node import Node

import time
from math import modf

from std_msgs.msg import Header
from custom_interfaces.msg import Controllermsg

import sys
sys.path.append('/home/volk/Desktop/DJI_Robomaster/env/ros_ws/src/robomaster_s1/robomaster_s1')
import ps5controller as controller


class controllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        #controller message
        self.controller_msg = Controllermsg()
        self.controller_msg.header = Header()
        self.controller_msg.header.frame_id = ''
        self.controller_msg.left_joystick = [0.0, 0.0]
        self.controller_msg.right_joystick = [0.0, 0.0]
        self.controller_msg.l2 = 0.0
        self.controller_msg.r2 = 0.0
        self.controller_msg.button = ''
        self.controller_msg.button_direction = ''

        #publisher
        self.controller_pub = self.create_publisher(Controllermsg, 'controller', 10)

        #logic params
        self.last_event = None
        self.last_publish_time = 0

    def publish_controller(self):
        self.controller_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.controller_msg.left_joystick, self.controller_msg.right_joystick = [float(self.scaled_left_joystick[0]), float(self.scaled_left_joystick[1])], [float(self.scaled_right_joystick[0]), float(self.scaled_right_joystick[1])]
        self.controller_msg.l2 = float(self.L2_scaled_value)
        self.controller_msg.r2 = float(self.R2_scaled_value)
        self.controller_msg.button = self.button
        self.controller_msg.button_direction = self.button_direction
        self.controller_pub.publish(self.controller_msg)

        self.last_publish_time = time.time()

    def run(self):
        while rclpy.ok():
            c1 = controller.ps5controller('/dev/input/event2')

            while True: #JV
                for event in c1.controller.read_loop():
                    c1.update_joystick_position(event)
                    # left_joystick, right_joystick = c1.get_joystick_position()
                    self.scaled_left_joystick = c1.linear_scaled_joystick('left',0,3.5)
                    self.scaled_right_joystick = c1.linear_scaled_joystick('right',0,100)

                    self.L2_value = c1.adaptive_trigger_press(event,'L2')
                    self.R2_value = c1.adaptive_trigger_press(event,'R2')
                    self.L2_scaled_value = c1.scaled_adaptive_trigger('L2',-300,0,True)
                    self.R2_scaled_value = c1.scaled_adaptive_trigger('R2',0,300,False)

                    self.button, self.button_direction = c1.get_any_button_press(event)


                    c1.update_joystick_position(event)
                    self.publish_controller()




def main(args=None):
    rclpy.init(args=args)

    controller_ros2 = controllerNode()
    controller_ros2.run()

    # Destroy the node explicitly
    controller_ros2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()