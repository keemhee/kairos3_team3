#!/usr/bin/env python3

import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymycobot.mycobot320 import MyCobot320
import numpy as np
import time


class topic_to_angle(Node):
    def __init__(self):
        super().__init__("topic_to_angle")
        self.get_logger().info("the node has started")
        self.mc = MyCobot320("/dev/ttyACM0", 115200)
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 30)
        self.sub_ = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 1)

    def joint_state_callback(self, msg):
        angles = [round(math.degrees(r)) for r in msg.position]
        temp = angles.pop(4)
        angles.insert(1, temp)
        gripper_raw = angles.pop()
        gripper = round((-gripper_raw / 46) * 100)
        self.get_logger().info(f"angle: {angles}, gripper: {gripper}")
        self.mc.send_angles(angles, 30)
        self.mc.set_gripper_value(gripper, 20)

def main(args=None):
    rclpy.init(args = args)
    node = topic_to_angle()
    rclpy.spin(node)
    rclpy.destroy_node(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
