#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP = "left_arm";

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node,
                                                            PLANNING_GROUP);

  const moveit::core::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  RCLCPP_INFO(LOGGER, "Planning frame: %s",
              move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(LOGGER, "End effector link: %s",
              move_group.getEndEffectorLink().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(),
            move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group,
                                         joint_group_positions);


//execute pos1 - START
  joint_group_positions[0] = -0.696; // Rotation 10
  joint_group_positions[1] = 0.425;  // Rotation 11
  joint_group_positions[2] = -0.357; // Rotation 12
  joint_group_positions[3] = -2.428; // Rotation 13
  move_group.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success =
      (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    move_group.execute(my_plan);
//execute pos1 - END
//execute pos2 - START
  joint_group_positions[0] = -1.783; // Rotation 10
  joint_group_positions[1] = 0.391;  // Rotation 11
  joint_group_positions[2] = -0.764; // Rotation 12
  joint_group_positions[3] = -0.153; // Rotation 13
  move_group.setJointValueTarget(joint_group_positions);

   success =
      (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    move_group.execute(my_plan);
//execute pos2 - END
//execute pos3 - START
  joint_group_positions[0] = -0.696; // Rotation 10
  joint_group_positions[1] = 0.425;  // Rotation 11
  joint_group_positions[2] = -0.357; // Rotation 12
  joint_group_positions[3] = -2.428; // Rotation 13
  move_group.setJointValueTarget(joint_group_positions);

   success =
      (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    move_group.execute(my_plan);
//execute pos3 - END



  rclcpp::shutdown();
  return 0;
}