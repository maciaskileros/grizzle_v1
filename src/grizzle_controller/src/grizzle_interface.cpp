#include "grizzle_controller/grizzle_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace grizzle_controller
{

std::string compensateZeros(const int value)
{
  std::string compensate_zeros = "";
  if(value < 10){
    compensate_zeros = "00";
  } else if(value < 100){
    compensate_zeros = "0";
  } else {
    compensate_zeros = "";
  }
  return compensate_zeros;
}
  
GrizzleInterface::GrizzleInterface()
{
}


GrizzleInterface::~GrizzleInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("GrizzleInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn GrizzleInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("GrizzleInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  position_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  prev_position_commands_.reserve(info_.joints.size());

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> GrizzleInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> GrizzleInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn GrizzleInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("GrizzleInterface"), "Starting robot hardware ...");

  // Reset commands and states
  position_commands_ = { 0.0, 0.0, 0.0, 0.0 };
  prev_position_commands_ = { 0.0, 0.0, 0.0, 0.0 };
  position_states_ = { 0.0, 0.0, 0.0, 0.0 };

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("GrizzleInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("GrizzleInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn GrizzleInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("GrizzleInterface"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("GrizzleInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("GrizzleInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

//read function is responsible for reading position of robot joints and make it available in ros2
//here we have space for improvemet - we can get feedback from servo position
hardware_interface::return_type GrizzleInterface::read(const rclcpp::Time &time,
                                                          const rclcpp::Duration &period)
{
  // Open Loop Control - assuming the robot is always where we command to be
  position_states_ = position_commands_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GrizzleInterface::write(const rclcpp::Time &time,
                                                           const rclcpp::Duration &period)
{
  if (position_commands_ == prev_position_commands_)
  {
    // Nothing changed, do not send any command
    return hardware_interface::return_type::OK;
  }

  std::string msg;
  int joint1 = static_cast<int>(((position_commands_.at(0) + (M_PI / 2)) * 180) / M_PI);
  msg.append("a");
  msg.append(compensateZeros(joint1));
  msg.append(std::to_string(joint1));
  msg.append(",");
  int joint2 = 180 - static_cast<int>(((position_commands_.at(1) + (M_PI / 2)) * 180) / M_PI);
  msg.append("b");
  msg.append(compensateZeros(joint2));
  msg.append(std::to_string(joint2));
  msg.append(",");
  int joint3 = static_cast<int>(((position_commands_.at(2) + (M_PI / 2)) * 180) / M_PI);
  msg.append("c");
  msg.append(compensateZeros(joint3));
  msg.append(std::to_string(joint3));
  msg.append(",");
  int joint4 = static_cast<int>(((-position_commands_.at(3)) * 180) / (M_PI / 2));
  msg.append("d");
  msg.append(compensateZeros(joint4));
  msg.append(std::to_string(joint4));
  msg.append(",");

  try
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("GrizzleInterface"), "Sending new command " << msg);
    arduino_.Write(msg);
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("GrizzleInterface"),
                        "Something went wrong while sending the message "
                            << msg << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  prev_position_commands_ = position_commands_;

  return hardware_interface::return_type::OK;
}
}  // namespace grizzle_controller

PLUGINLIB_EXPORT_CLASS(grizzle_controller::GrizzleInterface, hardware_interface::SystemInterface)
