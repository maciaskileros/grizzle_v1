#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SliderControl(Node):

    def __init__(self):
        super().__init__("slider_control")
        self.left_arm_pub_ = self.create_publisher(JointTrajectory, "left_arm_controller/joint_trajectory", 10)
        self.right_arm_pub_ = self.create_publisher(JointTrajectory, "right_arm_controller/joint_trajectory", 10)
        self.sub_ = self.create_subscription(JointState, "joint_commands", self.sliderCallback, 10)
        self.get_logger().info("Slider Control Node started")

    def sliderCallback(self, msg):
        left_arm_controller = JointTrajectory()
        right_arm_controller = JointTrajectory()
        left_arm_controller.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4"]
        right_arm_controller.joint_names = ["joint_5", "joint_6", "joint_7", "joint_8"]


        left_arm_goal = JointTrajectoryPoint()
        right_arm_goal = JointTrajectoryPoint()
        left_arm_goal.positions = msg.position[:4]
        right_arm_goal.positions = msg.position[4:8]

        left_arm_controller.points.append(left_arm_goal)
        right_arm_controller.points.append(right_arm_goal)
        
        self.left_arm_pub_.publish(left_arm_controller)
        self.right_arm_pub_.publish(right_arm_controller)


def main():
    rclpy.init()

    simple_publisher = SliderControl()
    rclpy.spin(simple_publisher)
    
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
