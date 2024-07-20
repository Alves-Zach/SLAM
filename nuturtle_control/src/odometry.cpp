#include <sstream>
#include <memory>
#include <chrono>
#include <string>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "turtlelib/diff_drive.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std;

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry"), count_(0)
  {
    // Parameters
    declare_parameter("body_id", rclcpp::PARAMETER_STRING);
    declare_parameter("odom_id", rclcpp::PARAMETER_STRING);
    declare_parameter("robot_params.left_wheel_joint", rclcpp::PARAMETER_STRING);
    declare_parameter("robot_params.right_wheel_joint", rclcpp::PARAMETER_STRING);
    declare_parameter("robot_params.wheel_radius", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.track_width", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.motor_cmd_max", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.motor_cmd_per_rad_sec", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.encoder_ticks_per_rad", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.collision_radius", rclcpp::PARAMETER_DOUBLE);
    pathDecimation = declare_parameter("path_decimation", 20);

    // Checking parameters have been initialized
    checkParameters();

    // Setting parameters
    /// \param body_id The name of the body frame of the robot
    body_id = get_parameter("body_id").as_string();
    /// \param wheel_left The name of the left wheel joint
    wheel_left = get_parameter("robot_params.left_wheel_joint").as_string();
    /// \param wheel_right The name of the right wheel joint
    wheel_right = get_parameter("robot_params.right_wheel_joint").as_string();
    /// \param wheel_radius the radius of the wheels on the turtlebot
    wheel_radius = get_parameter("robot_params.wheel_radius").as_double();
    /// \param track_width the distance between the center of the wheels
    track_width = get_parameter("robot_params.track_width").as_double();
    /// \param turtlebot the DiffDrive object that controls the turtlebot
    turtlebot = turtlelib::DiffDrive{get_parameter("robot_params.wheel_radius").as_double(),
      get_parameter("robot_params.track_width").as_double()};

    // Setting the constant atributes of internalOdom
    internalOdom.header.frame_id = "uncorrected_odom";
    internalOdom.child_frame_id = get_parameter("body_id").as_string();
    internalPose.covariance = {0};
    internalTwist.covariance = {0};

    // QOS object
    rclcpp::QoS markerQoS(10);
    markerQoS.transient_local();

    // Callback group
    auto defaultCBG = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);

    // Subscribers
    jointStateSubscriber = this->create_subscription \
      <sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(
        &Odometry::jointStateCallback,
        this, std::placeholders::_1));

    // Publishers
    odometryPublisher = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odometry",
      markerQoS);

    pathPublisher = this->create_publisher<nav_msgs::msg::Path>(
      "~/path",
      markerQoS);

    // Broadcaster
    tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Services
    initialPoseService = this->create_service<nuturtle_control::srv::InitialPose>(
      "~/initial_pose",
      std::bind(
        &Odometry::initialPoseCallback,
        this, std::placeholders::_1,
        std::placeholders::_2));

    // Initializing path object
    robotPath.header.frame_id = "nusim/world";

    // Setting path counter number
    pathCounter = 0;

    // Initializing transform objects
    correctedOdom_blue.header.frame_id = "uncorrected_odom";
    correctedOdom_blue.child_frame_id = "blue/base_footprint";
    correctedOdom_blue.transform.translation.z = 0.0;
  }

private:
    // Parameters
    /// \param body_id The name of the body frame of the robot
    string body_id;
    /// \param odom_id The name of the odometry frame
    string odom_id;
    /// \param wheel_left The name of the left wheel joint
    string wheel_left;
    /// \param wheel_right The name of the right wheel joint
    string wheel_right;
    /// \param wheel_radius the radius of the wheels on the turtlebot
    double wheel_radius;
    /// \param track_width the distance between the center of the wheels
    double track_width;
    /// \param turtlebot the DiffDrive object that controls the turtlebot
    turtlelib::DiffDrive turtlebot;
    double pathCounter, pathDecimation;

    // Count
    size_t count_;
    // The internal odometry message
    nav_msgs::msg::Odometry internalOdom;
    // The internal odometry pose
    geometry_msgs::msg::PoseWithCovariance internalPose;
    // The internal odometry twist
    geometry_msgs::msg::TwistWithCovariance internalTwist;
    // The internal odometry Twist2D
    turtlelib::Twist2D internalTwist2D{0, 0, 0};
    // The internal odometry twist
    tf2::Quaternion internalQuaternion{0, 0, 0, 0};
    // The stored joint state_message
    sensor_msgs::msg::JointState curJointStateMessage;
    // Robot's path
    nav_msgs::msg::Path robotPath;
    // Internal pose stamped
    geometry_msgs::msg::PoseStamped internalPoseStamped;
    // Current time stamp
    rclcpp::Time curTime;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher;

    // Broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber;
    void jointStateCallback(const sensor_msgs::msg::JointState & msg)
    {
        // Current time stamp
        curTime = this->get_clock()->now();

        // Store the current joint_state message in case initial pose is called
        curJointStateMessage = msg;

        // Figuring out twist from wheel velocities
        internalTwist2D = turtlebot.forwardKinematics(
        msg.velocity.at(0),
        msg.velocity.at(1));

        // Updating the stored odometry object
        setStoredOdom(
        turtlebot.position().rotation(),
        turtlebot.position().translation().x,
        turtlebot.position().translation().y);

        // Publish odom message
        odometryPublisher->publish(internalOdom);

        // Broadcast the odom_id to body_id transform
        uncorrectedOdom_blue_broadcaster(
        internalOdom.pose.pose.position.x,
        internalOdom.pose.pose.position.y,
        internalOdom.pose.pose.orientation);


        if (pathCounter == 0){
            // Update the blue robot's path
            blue_pathBroadcaster(
            internalOdom.pose.pose.position.x,
            internalOdom.pose.pose.position.y,
            internalOdom.pose.pose.orientation);
        }

        pathCounter++;

        if(pathCounter == pathDecimation){
            pathCounter = 0;
        }
    }

    // Services
    rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initialPoseService;
    void initialPoseCallback(
        std::shared_ptr<nuturtle_control::srv::InitialPose::Request> req,
        std::shared_ptr<nuturtle_control::srv::InitialPose::Response> res)
    {
        // Set the stored odometry to the requested position
        setStoredOdom(req->theta, req->x, req->y);

        // Reset the wheel positions of the blue turtlebot
        turtlebot.forwardKinematics(
        -curJointStateMessage.position.at(0),
        -curJointStateMessage.position.at(1));

        // Teleport the robot to the desired pose
        turtlebot.teleport(req->x, req->y, req->theta);

        // Set the response message to the current configuration
        res->success = true;
    }

    // Helper functions

    // Path updaters
    void blue_pathBroadcaster(double x, double y, geometry_msgs::msg::Quaternion q){
        // Only publishing if the robot moved
        if (!(turtlelib::almost_equal(x, internalPoseStamped.pose.position.x) &&
              turtlelib::almost_equal(y, internalPoseStamped.pose.position.y))){
                // Updating the internal pose
                internalPoseStamped.pose.position.x = x;
                internalPoseStamped.pose.position.y = y;
                internalPoseStamped.pose.orientation = q;

                // Stamping the pose
                internalPoseStamped.header.stamp = curTime;

                // Add a pose to the path message
                this->robotPath.poses.push_back(internalPoseStamped);
                // Send the nav_msgs path message
                robotPath.header.stamp = curTime;
                pathPublisher->publish(this->robotPath);
        }
    }
    
    // Transform Broadcasters
    geometry_msgs::msg::TransformStamped correctedOdom_blue;
    void uncorrectedOdom_blue_broadcaster(double x, double y, geometry_msgs::msg::Quaternion orientation)
    {
        // Read message content and assign it to
        // corresponding tf variables
        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        correctedOdom_blue.transform.translation.x = x;
        correctedOdom_blue.transform.translation.y = y;

        correctedOdom_blue.transform.rotation = orientation;

        correctedOdom_blue.header.stamp = curTime;

        // Send the transformation
        tfBroadcaster->sendTransform(correctedOdom_blue);
    }

    void setStoredOdom(double thetaIn, double xIn, double yIn)
    {
        // Set the stored pose to the requested pose
        internalPose.pose.position.set__x(xIn);
        internalPose.pose.position.set__y(yIn);

        // Creating a quaternion to enter into internalPose.orientation
        internalQuaternion.setRPY(0.0, 0.0, thetaIn);
        internalPose.pose.orientation.set__w(internalQuaternion.w());
        internalPose.pose.orientation.set__x(internalQuaternion.x());
        internalPose.pose.orientation.set__y(internalQuaternion.y());
        internalPose.pose.orientation.set__z(internalQuaternion.z());

        // Set the stored Twist
        internalTwist.twist.linear.set__x(xIn);
        internalTwist.twist.linear.set__y(yIn);
        internalTwist.twist.angular.set__z(thetaIn);

        // Set the stored Twist2D
        internalTwist2D.x = xIn;
        internalTwist2D.y = yIn;
        internalTwist2D.omega = thetaIn;

        // Set message twist
        internalOdom.twist = internalTwist;
        // Set message pose
        internalOdom.pose = internalPose;
    }

    void checkParameters()
    {
        // List of parameters to check
        const vector<string> paramList = {"robot_params.wheel_radius", "robot_params.track_width", \
        "robot_params.motor_cmd_max", "robot_params.motor_cmd_per_rad_sec", \
        "robot_params.encoder_ticks_per_rad", "robot_params.collision_radius", \
        "body_id", "odom_id", "robot_params.left_wheel_joint", "robot_params.right_wheel_joint"};
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
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
