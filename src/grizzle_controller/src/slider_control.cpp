#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <chrono>


using namespace std::chrono_literals;
using std::placeholders::_1;

class SliderControl : public rclcpp::Node
{
public:
  SliderControl() : Node("slider_control")
  {
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_commands", 10, std::bind(&SliderControl::sliderCallback, this, _1));
    left_arm_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("left_arm_controller/joint_trajectory", 10);
    right_arm_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("right_arm_controller/joint_trajectory", 10);
    RCLCPP_INFO(get_logger(), "Slider Control Node started");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_arm_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_arm_pub_;

  void sliderCallback(const sensor_msgs::msg::JointState &msg) const
  {
    trajectory_msgs::msg::JointTrajectory left_arm_command, right_arm_command;
    right_arm_command.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4"};
    left_arm_command.joint_names = {"joint_5", "joint_6", "joint_7", "joint_8"};

    trajectory_msgs::msg::JointTrajectoryPoint left_arm_goal, right_arm_goal;
    left_arm_goal.positions.insert(left_arm_goal.positions.end(), msg.position.begin(), msg.position.begin() + 4);
    right_arm_goal.positions.insert(right_arm_goal.positions.end(), msg.position.begin()+4, msg.position.begin() + 8);

    left_arm_command.points.push_back(left_arm_goal);
    right_arm_command.points.push_back(right_arm_goal);

    left_arm_pub_->publish(left_arm_command);
    right_arm_pub_->publish(right_arm_command);
  }
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SliderControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
