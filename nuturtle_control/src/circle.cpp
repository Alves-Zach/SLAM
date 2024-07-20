#include <sstream>
#include <memory>
#include <chrono>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "nuturtle_control/srv/straight.hpp"

using namespace std;

/// \brief Node that controls a turtlebot
class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle"), count_(0)
  {
    // Paramaters
    declare_parameter("rate", 200.0);
    declare_parameter("robot_params.wheel_radius", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.track_width", rclcpp::PARAMETER_DOUBLE);

    // Checking that all parameters have been initialized
    checkParameters();

    // Setting parameters
    /// \param wheel_radius the radius of the wheels on the turtlebot
    wheel_radius = get_parameter("robot_params.wheel_radius").as_double();
    /// \param track_width the distance between the center of the wheels
    track_width = get_parameter("robot_params.track_width").as_double();
    /// \param turtlebot the DiffDrive object that controls the turtlebot
    turtlebot = turtlelib::DiffDrive{wheel_radius, track_width};

    // Timer
    timer =
      this->create_wall_timer(
      1s / get_parameter("rate").as_double(),
      std::bind(&Circle::timer_callback, this));

    // Publishers
    cmd_velPublisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Services
    control = this->create_service<nuturtle_control::srv::Control>(
      "~/control",
      std::bind(
        &Circle::controlCallback,
        this, std::placeholders::_1,
        std::placeholders::_2));
    straight = this->create_service<nuturtle_control::srv::Straight>(
      "~/straight",
      std::bind(
        &Circle::straightCallback,
        this, std::placeholders::_1,
        std::placeholders::_2));
    reverse = this->create_service<std_srvs::srv::Empty>(
      "~/reverse",
      std::bind(
        &Circle::reverseCallback,
        this, std::placeholders::_1,
        std::placeholders::_2));
    stop = this->create_service<std_srvs::srv::Empty>(
      "~/stop",
      std::bind(
        &Circle::stopCallback,
        this, std::placeholders::_1,
        std::placeholders::_2));
  }

private:
  // Parameters
  /// \param wheel_radius the radius of the wheels on the turtlebot
  double wheel_radius;
  /// \param track_width the distance between the center of the wheels
  double track_width;
  /// \param turtlebot the DiffDrive object that controls the turtlebot
  turtlelib::DiffDrive turtlebot;       // Has wheelTrack and wheelRadius
  /// \param state Boolean that corrisponds to the state of the system
  int state = 0;       // 0 = stopped and 1 = moving
  /// \param curTwist the current twist of the robot
  geometry_msgs::msg::Twist curTwist;


  // Timer callback for this node
  rclcpp::TimerBase::SharedPtr timer;
  void timer_callback()
  {
    if (state == 1) {       // State is moving
      // Constantly publish the cmd_vel message
      cmd_velPublisher->publish(curTwist);
    }
    // Else do nothing
  }

  // Count
  size_t count_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_velPublisher;

  // Services

  // Make the robot start moving in a circle of defined radius and at defined speed
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control;
  void controlCallback(
    std::shared_ptr<nuturtle_control::srv::Control::Request> req,
    std::shared_ptr<nuturtle_control::srv::Control::Response> res)
  {
    // Set state to moving
    state = 1;

    curTwist.angular.z = req->velocity;
    curTwist.linear.x = req->velocity * req->radius;

    // Setting response to success
    res->success = "Success";
  }

  // Make the robot start moving in a circle of defined radius and at defined speed
  rclcpp::Service<nuturtle_control::srv::Straight>::SharedPtr straight;
  void straightCallback(
    std::shared_ptr<nuturtle_control::srv::Straight::Request> req,
    std::shared_ptr<nuturtle_control::srv::Straight::Response> res)
  {
    // Set state to moving
    state = 1;

    curTwist.angular.z = 0.0;
    curTwist.linear.x = req->velocity;

    // Setting response to success
    res->success = "Success";
  }
  
  // Reverse the robot's direction of travel
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse;
  void reverseCallback(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // Set state to moving
    state = 1;

    // Reverse the direction of the robot
    curTwist.angular.z = -curTwist.angular.z;
    curTwist.linear.x = -curTwist.linear.x;
  }

  // Make the robot stop moving around the circle
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop;
  void stopCallback(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // Set state to stopped
    state = 0;

    // Updating curTwist to stop the robot
    curTwist.angular.z = 0;
    curTwist.linear.x = 0;

    cmd_velPublisher->publish(curTwist);
  }

  // Helper functions
  void checkParameters()
  {
    // List of parameters to check
    const vector<string> paramList = {"robot_params.wheel_radius", "robot_params.track_width"};
    long unsigned i = 0;
    // Checking that all parameters have been initialized
    try {
      for (i = 0; i < paramList.size(); i++) {
        get_parameter(paramList.at(i));
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
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
