#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtle_control/msg/twist.hpp"
#include "turtlelib/diff_drive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/callback_group.hpp"

using namespace std;

/// \brief Node that controls a turtlebot
class turtle_control : public rclcpp::Node
{
public:
  turtle_control()
  : Node("turtle_control"), count_(0)
  {
    // Paramaters
    declare_parameter("robot_params.wheel_radius", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.track_width", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.motor_cmd_max", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.motor_cmd_per_rad_sec", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.encoder_ticks_per_rad", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.collision_radius", rclcpp::PARAMETER_DOUBLE);

    // Checking that all parameters have been initialized
    checkParameters();

    // Setting parameters
    wheel_radius = get_parameter("robot_params.wheel_radius").as_double();
    track_width = get_parameter("robot_params.track_width").as_double();
    turtlebot = turtlelib::DiffDrive{wheel_radius, track_width};

    // Setting up the joint state message
    jointStateOutputMessage.name = {"wheel_left_joint", "wheel_right_joint"};

    // Publishers
    wheelCommandPublisher = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
      "/wheel_cmd", 10);
    redJointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>("red/joint_states", 10);
    blueJointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>("blue/joint_states", 10);
    greenJointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>("green/joint_states", 10);

    // Subscribers
    cmd_velSubscriber = this->create_subscription \
      <geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(
        &turtle_control::cmd_velCallback,
        this, std::placeholders::_1));
    sensor_dataSubscriber = this->create_subscription \
      <nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(
        &turtle_control::sensor_dataCallback,
        this, std::placeholders::_1));
  }

private:
  // Parameters
  /// \param wheel_radius the radius of the wheels on the turtlebot
  double wheel_radius;
  /// \param track_width the distance between the center of the wheels
  double track_width;
  /// \param turtlebot the DiffDrive object that controls the turtlebot
  turtlelib::DiffDrive turtlebot;

  // Count
  size_t count_;

  // Joint_state message
  sensor_msgs::msg::JointState jointStateOutputMessage;

  // First sensor_data flag
  bool firstSensorData = true;

  // Publishers
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheelCommandPublisher;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr redJointStatePublisher;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr blueJointStatePublisher;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr greenJointStatePublisher;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_velSubscriber;
  void cmd_velCallback(const geometry_msgs::msg::Twist msg)
  {
    // Convert the geometry_msgs twist into a twist from turtlelib
    const turtlelib::Twist2D libTwist{msg.angular.z, msg.linear.x, msg.linear.y};

    // Turn the twist input into wheel speed outputs
    vector<double> calcWheelCommands = this->turtlebot.inverseKinematics(libTwist);

    // Creating a message from the two wheel velocities
    // Converting from rad/s to mcu
    nuturtlebot_msgs::msg::WheelCommands outputMessage;
    outputMessage.left_velocity = calcWheelCommands.at(0) / get_parameter(
      "robot_params.motor_cmd_per_rad_sec").as_double();
    outputMessage.right_velocity = calcWheelCommands.at(1) / get_parameter(
      "robot_params.motor_cmd_per_rad_sec").as_double();

    // Sending wheel commands to the wheel_cmd topic
    wheelCommandPublisher->publish(outputMessage);
  }

  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_dataSubscriber;
  void sensor_dataCallback(const nuturtlebot_msgs::msg::SensorData msg)
  {
    // Check if this is the first sensor_data message
    if (firstSensorData == true) {
      // publish joint states of {0, 0} for pos and vel
      jointStateOutputMessage.position = {0.0, 0.0};
      jointStateOutputMessage.velocity = {0.0, 0.0};

      // Set flag to false
      firstSensorData = false;
    } else {
      // Figure out position and velocity based on:
      //Calculating position (msg) (location/encoder_ticks_per_rad)
      double leftWheelPosition = msg.left_encoder /
        (get_parameter("robot_params.encoder_ticks_per_rad").as_double());
      double rightWheelPosition = msg.right_encoder /
        (get_parameter("robot_params.encoder_ticks_per_rad").as_double());

      // Calculating velocity
      double leftWheelVeclocity = (leftWheelPosition - jointStateOutputMessage.position[0]);
      double rightWheelVeclocity = (rightWheelPosition - jointStateOutputMessage.position[1]);

      // Setting position for jointStateMessage
      jointStateOutputMessage.position = {leftWheelPosition, rightWheelPosition};

      // Setting velocity for jointStateMessage
      jointStateOutputMessage.velocity = {leftWheelVeclocity, rightWheelVeclocity};
    }

    // get current time then publish the message
    jointStateOutputMessage.header.stamp = this->get_clock()->now();
    blueJointStatePublisher->publish(jointStateOutputMessage);
    redJointStatePublisher->publish(jointStateOutputMessage);
    greenJointStatePublisher->publish(jointStateOutputMessage);
  }

  // Helper functions
  void checkParameters()
  {
    // List of parameters to check
    const vector<string> paramList = {"robot_params.wheel_radius", "robot_params.track_width", \
      "robot_params.motor_cmd_max", "robot_params.motor_cmd_per_rad_sec", \
      "robot_params.encoder_ticks_per_rad", "robot_params.collision_radius"};
    long unsigned i = 0;
    // Checking that all parameters have been initialized
    try {
      for (i = 0; i < paramList.size(); i++) {
        get_parameter("" + paramList.at(i));
      }
    } catch (rclcpp::exceptions::ParameterUninitializedException &) {
      // Create error message
      string errorString = "Parameter(" + paramList.at(i);
      errorString += ") has not been initialized and is needed";

      // Throw an exception that i param wasn't initialized
      RCLCPP_ERROR_STREAM(get_logger(), errorString);
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtle_control>());
  rclcpp::shutdown();
  return 0;
}
