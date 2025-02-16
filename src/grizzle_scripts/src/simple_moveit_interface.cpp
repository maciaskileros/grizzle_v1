#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>


void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    auto left_arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "left_arm");
    auto right_arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "right_arm");

    std::vector<double> right_arm_joint_goal {0.628, 0.254, -0.391, 2.496};
    std::vector<double> left_arm_joint_goal {-0.696, 0.425, -0.357, -2.462};

    bool left_arm_within_bounds = left_arm_move_group.setJointValueTarget(left_arm_joint_goal);
    bool right_arm_within_bounds = right_arm_move_group.setJointValueTarget(right_arm_joint_goal);

    if (!left_arm_within_bounds | !right_arm_within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan left_arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan right_arm_plan;
    bool left_arm_plan_success = (left_arm_move_group.plan(left_arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool right_arm_plan_success = (right_arm_move_group.plan(right_arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(left_arm_plan_success && right_arm_plan_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Planner SUCCEED, moving the left_arm and the rgiht_arm");
        left_arm_move_group.move();
        right_arm_move_group.move();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "One or more planners failed!");
        return;
    }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");
  move_robot(node);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
}