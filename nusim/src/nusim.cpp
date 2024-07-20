#include <sstream>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <string.h>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rclcpp/callback_group.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#define PI 3.14159

using namespace std::chrono_literals;

/// \brief Node that controls all of the simulation's frames and objects
class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"), count_(0)
  {
    // *******************Parameters******************* //
    // Parameters that change the node's behavior
    std::string draw_only = declare_parameter("draw_only", "false");
    double rate = declare_parameter("rate", 200.0);
    // Robot position params
    double x0 = declare_parameter("x0", 0.0);
    double y0 = declare_parameter("y0", 0.0);
    double theta0 = declare_parameter("theta0", 0.0);
    // Robot behavior params
    declare_parameter("robot_params.motor_cmd_per_rad_sec", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.encoder_ticks_per_rad", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.wheel_radius", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.track_width", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("robot_params.collision_radius", rclcpp::PARAMETER_DOUBLE);
    // LiDAR params
    declare_parameter("robot_params.min_lidar_range", 0.11999999731779099);
    declare_parameter("robot_params.max_lidar_range", 3.5);
    declare_parameter("robot_params.lidar_angular_increment", 0.01745329238474369);
    declare_parameter("robot_params.number_of_samples", 5.0);
    declare_parameter("robot_params.lidar_resolution", 0.01745329238474369);
    declare_parameter("robot_params.lidar_noise", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("use_lidar", true);
    // Arena params
    declare_parameter("arena_x_length", 10.0);
    declare_parameter("arena_y_length", 10.0);
    std::vector<double> obsX = declare_parameter("obsticles/x", std::vector<double>{-0.5, 0.8, 0.4});
    std::vector<double> obsY = declare_parameter("obsticles/y", std::vector<double>{-0.7, -0.8, 0.8});
    declare_parameter("obsticles/r", 0.038);
    // Simulation params
    declare_parameter("input_noise", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("slip_fraction", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("basic_sensor_max_range", 2.0); // *Changed from 1.0*
    declare_parameter("basic_sensor_variance", rclcpp::PARAMETER_DOUBLE);
    pathDecimation = declare_parameter("path_decimation", 20);

    // Error checking for x and y vectors
    if (obsX.size() != obsY.size()) {
      // Throw an error and exit
      RCLCPP_ERROR_STREAM(get_logger(), "Not a valid input for X and Y obsticle locations");
      exit(0);
    }

    // Initializing parameters
    /// \param turtlebot the DiffDrive object that controls the turtlebot
    turtlebot = turtlelib::DiffDrive{get_parameter("robot_params.wheel_radius").as_double(),
      get_parameter("robot_params.track_width").as_double()};

    // QOS object
    rclcpp::QoS markerQoS(10);
    markerQoS.transient_local();

    // Callback group
    auto defaultCBG = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);


    // *******************Timers******************* //
    // Timer
    timer = this->create_wall_timer(1s / rate, std::bind(&Nusim::timer_callback, this));

    // Sensor timer
    sensorTimer = this->create_wall_timer(1s / 5.0, std::bind(&Nusim::sensor_timer_callback, this));

    // *******************Subscribers******************* //
    redWheelCmdSubscriber = this->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "/wheel_cmd",
      10,
      std::bind(&Nusim::redWheelCmdCallback, this, std::placeholders::_1));

    // *******************Publishers******************* //
    timestepPublisher = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    wallPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/walls",
      markerQoS);
    obsticlePublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obsticles",
      markerQoS);
    fakeSensorPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fake_sensor",
      markerQoS);
    sensor_dataPublisher = this->create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "/sensor_data",
      10);
    pathPublisher = this->create_publisher<nav_msgs::msg::Path>(
      "~/path",
      markerQoS);
    laserScanPublisher = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "/laser_scan",
      markerQoS);

    // *******************Broadcaster******************* //
    world_red = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // *******************Services******************* //
    reset = this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(
        &Nusim::reset_callback,
        this, std::placeholders::_1,
        std::placeholders::_2));
    teleport = this->create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(
        &Nusim::teleport_callback,
        this, std::placeholders::_1,
        std::placeholders::_2));

    // *******************TF buffer and listener******************* //
    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    // World to red/base_frame transform
    world_redTransform.header.frame_id = "nusim/world";
    world_redTransform.child_frame_id = "red/base_footprint";
    world_redTransform.transform.translation.z = 0.0;

    // Creating the arena
    createWalls();
    createObsticles();

    // Creating the detected obsticle markers
    createDetectedObsticle();

    if (!get_parameter("use_lidar").as_bool()){
        // Publish the marker array again
        fakeSensorPublisher->publish(detectedObsticleArray);
    }

    // *******************Initializing messages******************* //
    // Path message
    robotPath.header.frame_id = "nusim/world";

    // Laser Scan message
    laserScanMessage.header.frame_id = "red/base_scan";
    laserScanMessage.angle_min = 0.0;
    laserScanMessage.angle_max = 6.2657318115234375;
    laserScanMessage.angle_increment = get_parameter("robot_params.lidar_angular_increment").as_double();
    laserScanMessage.time_increment = 0.0;
    laserScanMessage.scan_time = 1.0 / 5.0;
    laserScanMessage.range_min = get_parameter("robot_params.min_lidar_range").as_double();
    laserScanMessage.range_max = get_parameter("robot_params.max_lidar_range").as_double();
    laserScanMessage.ranges.resize(360);
        // int(6.2657318115234375 / get_parameter("robot_params.lidar_angular_increment").as_double()));

    // Putting the robot at the initial position
    // Find the wheel commands that would get the blue robot to the desired position
    std::vector<double> wheelVelCommands = turtlebot.inverseKinematics(turtlelib::Twist2D{theta0,
                                                                                          x0,
                                                                                          y0});

    // Set the position of the red robot
    turtlebot.forwardKinematics(wheelVelCommands.at(0), wheelVelCommands.at(1));

    // Set the posiiton of the simulated robot
    world_red_broadcaster(
      turtlebot.position().translation().x,
      turtlebot.position().translation().y,
      turtlebot.position().rotation());

    // *******************Setting the random number generator's seed******************* //
    generator.seed(12345);

    // Setting path counter number
    pathCounter = 0;
  }

private:
// *******************Parameters******************* //
// Count
  size_t count_;

// Simulation variables
  int timestep = 0;
  rclcpp::Time curTimerTime, curSensorTime;
  visualization_msgs::msg::MarkerArray obsticleMarkArray;
  std::vector<turtlelib::Transform2D> obsticleTransforms; // Transforms are WORLD TO OBSTICLE
  nav_msgs::msg::Path robotPath;
  std_msgs::msg::UInt64 timeStepMessage;
  nuturtlebot_msgs::msg::SensorData sensor_dataMessage;
  turtlelib::DiffDrive turtlebot;

  // Variable used to decimate path position publishing
  int pathCounter, pathDecimation;

// Current position and velocity
  geometry_msgs::msg::PoseStamped curPoseStamped;
  std::vector<double> wheelVelocities{0.0, 0.0};
  std::vector<double> wheelPositions{0.0, 0.0};

// LiDAR variables
  double curAngle = 0.0;
  double curR; // The radius of the current obsticle
  double sgn;
  sensor_msgs::msg::LaserScan laserScanMessage;
  int curLidarVectorIndex = 0;

// World to red transform variables
  geometry_msgs::msg::TransformStamped world_redTransform;
  tf2::Quaternion world_redQuaternion;

// Fake sensor obsticle checker variables
  std::normal_distribution<double> basicSensorNoise{0.0, 0.333};
  std::uniform_real_distribution<double> uniformRandomAngle{-PI, PI};
  double randAngle, randDistance;

// Detected obsticle marker array and transform array
  visualization_msgs::msg::MarkerArray detectedObsticleArray;
  std::vector<turtlelib::Transform2D> detectedObsticleTransforms; // Transforms are TURTLE TO OBSTICLE

// Variables used to detect circle line collision
  turtlelib::Transform2D lowRangePoint_scan, highRangePoint_scan;
  turtlelib::Transform2D lowRangePoint_obs, highRangePoint_obs;
  turtlelib::Transform2D intersect1_obs, intersect2_obs;
  turtlelib::Transform2D intersect1_scan, intersect2_scan;
  turtlelib::Transform2D intersect_scanActual;
  turtlelib::Transform2D curBaseScanFrame2D;
  double noiseAddedLidarRange;
  double dx, dy, dr, D, delta;

// Lidar noise
  std::normal_distribution<double> lidarNoise{0.0, 0.333};

// Lidar to wall variables
  turtlelib::Transform2D lowRangePoint_world, highRangePoint_world;
  turtlelib::Transform2D wallPointWorld, wallPointScan;
  turtlelib::Vector2D wallPointScanVec;
  double wallLidarRange, curLaserSlope;

// Random number generator for cmdVel callback to use
  std::default_random_engine generator;
  std::normal_distribution<double> wheelNoiseDist{0.0, 0.333};
  std::uniform_real_distribution<double> slipDist{-1, 1};

// TF listener and buffer
  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};

// Wall markers
  visualization_msgs::msg::MarkerArray wallMarkArray;
  visualization_msgs::msg::Marker nWall;
  visualization_msgs::msg::Marker eWall;
  visualization_msgs::msg::Marker sWall;
  visualization_msgs::msg::Marker wWall;

    // *******************Timers******************* //
// Main timer
  rclcpp::TimerBase::SharedPtr timer;
  void timer_callback(){
    // Incrementing timestep
    timestep++;

    // Getting the current time as a variables
    curTimerTime = this->get_clock()->now();

    // Updating the message to publish to the timeStep topic
    timeStepMessage.data = timestep;

    // Publishing the timestepMessage
    timestepPublisher->publish(timeStepMessage);

    // Updating wheel positions
    wheelPositions.at(0) += wheelVelocities.at(0) / get_parameter("rate").as_double();
    wheelPositions.at(1) += wheelVelocities.at(1) / get_parameter("rate").as_double();

    // Finding the new position of the robot
    turtlebot.forwardKinematics(
      wheelVelocities.at(0) / get_parameter("rate").as_double(),
      wheelVelocities.at(1) / get_parameter("rate").as_double());


    // Check obstacle colission
    checkObstacleCollision();

    // Checking wall collission
    checkWallCollision();

    // Updating the sensor_dataMessage
    sensor_dataMessage.stamp = curTimerTime;
    sensor_dataMessage.left_encoder = wheelPositions.at(0) * get_parameter(
      "robot_params.encoder_ticks_per_rad").as_double();
    sensor_dataMessage.right_encoder = wheelPositions.at(1) * get_parameter(
      "robot_params.encoder_ticks_per_rad").as_double();

    // Publishing the sensor_dataMessage
    sensor_dataPublisher->publish(sensor_dataMessage);

    // Publish the new location of the robot
    world_red_broadcaster(
      turtlebot.position().translation().x,
      turtlebot.position().translation().y,
      turtlebot.position().rotation());

    if (pathCounter == 0){
        // Updating the robot path
        pathBroadcaster(
            turtlebot.position().translation().x,
            turtlebot.position().translation().y,
            turtlebot.position().rotation());
    }

    pathCounter++;

    if(pathCounter == pathDecimation){
        pathCounter = 0;
    }
  }

// Sensor timer
  rclcpp::TimerBase::SharedPtr sensorTimer;
  void sensor_timer_callback(){
    // Getting current sensor time as a variable
    curSensorTime = this->get_clock()->now();

    // Loop checking through each obsticle
    for (long unsigned int i = 0; i < detectedObsticleArray.markers.size(); i++) {
        // Checking if the fake sensor hit obsticles
        checkFakeSensorObsticles(i);
    }

    for (long unsigned int j = 0; j < laserScanMessage.ranges.size(); j++){
        // Set the previous value to zero to clear previous measurements
        laserScanMessage.ranges.at(j) = 0.0;

        for (long unsigned int i = 0; i < detectedObsticleArray.markers.size(); i++) {
            // Checking if the lidar hit the obsticles
            checkLidarToObsticles(i);
        }

        // Check for the lidar hitting the walls
        checkLidarToWalls();

        // Increment the laser scanner
        curAngle += laserScanMessage.angle_increment;

        // Incrementing the lidar index value
        curLidarVectorIndex++;
        if (curLidarVectorIndex >= (int)(laserScanMessage.ranges.size())){
            curLidarVectorIndex = 0;
        }
    }

    curAngle = 0.0;

    // Only publish the fake sensor if use_lidar is false
    if (!get_parameter("use_lidar").as_bool()){
        // Publish the marker array again
        fakeSensorPublisher->publish(detectedObsticleArray);
    }

    // Publish the laserScan message
    laserScanMessage.header.stamp = curSensorTime;
    laserScanPublisher->publish(laserScanMessage);
}

// *******************Services******************* //

// Reset the simulation's state
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset;
  void reset_callback(std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // Reset the timestep
    timestep = 0;
    
    // Set the wheel rotations to 0, 0
    turtlebot.forwardKinematics(-wheelPositions.at(0), -wheelPositions.at(1));

    // Teleport the robot back to 0, 0, 0
    turtlebot.teleport(0.0, 0.0, 0.0);

    // Set the posiiton of the simulated robot
    world_red_broadcaster(
      turtlebot.position().translation().x,
      turtlebot.position().translation().y,
      turtlebot.position().rotation());
  }

// Teleport the robot to a new location
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport;
  void teleport_callback(nusim::srv::Teleport::Request::SharedPtr req, nusim::srv::Teleport::Response::SharedPtr res)
  {
    // Call the robot's teleport function
    turtlebot.teleport(req->x, req->y, req->theta);

    // Set the posiiton of the simulated robot
    world_red_broadcaster(
      turtlebot.position().translation().x,
      turtlebot.position().translation().y,
      turtlebot.position().rotation());

    res->success = true;
  }

// *******************Helper functions******************* //
// Transform Broadcaster helper function
  void world_red_broadcaster(double x, double y, double w)
  {
    // Read message content and assign it to
    // corresponding tf variables
    world_redTransform.header.stamp = curTimerTime;

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    world_redTransform.transform.translation.x = x;
    world_redTransform.transform.translation.y = y;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    world_redQuaternion.setRPY(0, 0, w);
    world_redTransform.transform.rotation.x = world_redQuaternion.x();
    world_redTransform.transform.rotation.y = world_redQuaternion.y();
    world_redTransform.transform.rotation.z = world_redQuaternion.z();
    world_redTransform.transform.rotation.w = world_redQuaternion.w();

    // Send the transformation
    world_red->sendTransform(world_redTransform);
  }

// path broadcaster helpper function
  void pathBroadcaster(double x, double y, double w){
    // Only publishing if the robot moved
    if (!(turtlelib::almost_equal(x, curPoseStamped.pose.position.x) &&
          turtlelib::almost_equal(y, curPoseStamped.pose.position.y))){

        // Making a quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, w);

        // Updating current pose information
        curPoseStamped.pose.position.x = x;
        curPoseStamped.pose.position.y = y;
        curPoseStamped.pose.orientation.x = q.x();
        curPoseStamped.pose.orientation.y = q.y();
        curPoseStamped.pose.orientation.z = q.z();
        curPoseStamped.pose.orientation.w = q.w();

        // Stamping the pose
        curPoseStamped.header.stamp = curTimerTime;

        // Add a pose to the path message
        this->robotPath.poses.push_back(curPoseStamped);
        // Send the nav_msgs path message
        robotPath.header.stamp = curTimerTime;
        pathPublisher->publish(this->robotPath);
    }
  }

// Detected obsticle helper functions
  // Creates the detected objects
  void createDetectedObsticle(){
    // Object parameters
    double obsticleR = get_parameter("obsticles/r").as_double();
    double obsticleH = 0.25;
    std::vector<double> obsticleX = get_parameter("obsticles/x").as_double_array();
    std::vector<double> obsticleY = get_parameter("obsticles/y").as_double_array();
    visualization_msgs::msg::Marker obsticle;

    // Obsticle loop
    for (long unsigned int i = 0; i < obsticleX.size(); i++) {
        // Create the detected obsticle
        obsticle.header.frame_id = "red/base_footprint";
        obsticle.header.stamp = this->get_clock()->now();
        obsticle.id = detectedObsticleArray.markers.size();
        obsticle.type = visualization_msgs::msg::Marker::CYLINDER;
        obsticle.action = visualization_msgs::msg::Marker::ADD;
        obsticle.scale.x = obsticleR * 2;
        obsticle.scale.y = obsticleR * 2;
        obsticle.scale.z = 1.25 * obsticleH;
        obsticle.pose.position.x = obsticleX.at(i);
        obsticle.pose.position.y = obsticleY.at(i);
        obsticle.pose.position.z = 1.25 * obsticleH / 2;
        obsticle.pose.orientation.x = 0.0;
        obsticle.pose.orientation.y = 0.0;
        obsticle.pose.orientation.z = 0.0;
        obsticle.pose.orientation.w = 0.0;
        obsticle.color.r = 1.0;
        obsticle.color.g = 1.0;
        obsticle.color.b = 28.0/255.0;
        obsticle.color.a = 0.75;

        if (distanceToMarker(obsticle) <= get_parameter("basic_sensor_max_range").as_double()){
            obsticle.action = visualization_msgs::msg::Marker::ADD;
        }else{
            obsticle.action = visualization_msgs::msg::Marker::DELETE;
        }

        detectedObsticleArray.markers.push_back(obsticle);

        // Creating the transform that describes the obsticle 
        // This transform is FROM TURTLE TO OBSTICLE
        turtlelib::Transform2D trans = turtlebot.position().inv() * obsticleTransforms.at(obsticleTransforms.size() - 1);

        detectedObsticleTransforms.push_back(trans);
    }
  }

    // Helper function to check for lidar obsticle colissions given the obsticle's index
void checkFakeSensorObsticles(int i){
    // Check if any of the obsticles are out of range
    if (distanceToMarker(obsticleMarkArray.markers.at(i)) > get_parameter("basic_sensor_max_range").as_double()){
        // Set the marker's action to DELETE so it gets hidden
        detectedObsticleArray.markers.at(i).action = visualization_msgs::msg::Marker::DELETE;
    }

    // Check if any of the obsticles are in range
    if (distanceToMarker(obsticleMarkArray.markers.at(i)) <= get_parameter("basic_sensor_max_range").as_double() && 
        detectedObsticleArray.markers.at(i).action == visualization_msgs::msg::Marker::DELETE){
        // Set the marker's action to ADD so it gets shown
        detectedObsticleArray.markers.at(i).action = visualization_msgs::msg::Marker::ADD;
    }

    // Edit the transform to be at the right location relative to the robot
    detectedObsticleTransforms.at(i) = turtlebot.position().inv() * obsticleTransforms.at(i);
    // Edit the marker to follow the transform
    detectedObsticleArray.markers.at(i).pose.position.x = detectedObsticleTransforms.at(i).translation().x;
    detectedObsticleArray.markers.at(i).pose.position.y = detectedObsticleTransforms.at(i).translation().y;

    // Adding noise to the position of the detected obsticle
    randAngle = uniformRandomAngle(generator);
    randDistance = get_parameter("basic_sensor_variance").as_double() * basicSensorNoise(generator);
    detectedObsticleArray.markers.at(i).pose.position.x += randDistance * cos(randAngle);
    detectedObsticleArray.markers.at(i).pose.position.y += randDistance * sin(randAngle);

    // Setting the marker's stamp
    detectedObsticleArray.markers.at(i).header.stamp = curSensorTime;
}

// Checks if the lidar hit the obsticles given the obsticle's index
void checkLidarToObsticles(int i){
    // Getting the current radius of the current obsticle
    curR = get_parameter("obsticles/r").as_double();

    // Getting the current scanner frame
    curBaseScanFrame2D = turtlebot.position() * turtlelib::Transform2D{turtlelib::Vector2D{-0.032, 0.0}, curAngle};

    // Creating points at the low and high ranges of the lidar
    // Tsr
    lowRangePoint_scan = {turtlelib::Vector2D{get_parameter("robot_params.min_lidar_range").as_double(), 0.0}, 0.0};
    highRangePoint_scan = {turtlelib::Vector2D{get_parameter("robot_params.max_lidar_range").as_double(), 0.0}, 0.0};

    // Converting points to obsticle frame
    // Tor = Two.inv() * Tws * Tsr
    lowRangePoint_obs = obsticleTransforms.at(i).inv() * curBaseScanFrame2D * lowRangePoint_scan;
    highRangePoint_obs = obsticleTransforms.at(i).inv() * curBaseScanFrame2D * highRangePoint_scan;

    dx = highRangePoint_obs.translation().x - lowRangePoint_obs.translation().x;
    dy = highRangePoint_obs.translation().y - lowRangePoint_obs.translation().y;
    dr = sqrt(pow(dx, 2) + pow(dy, 2));

    D = (lowRangePoint_obs.translation().x * highRangePoint_obs.translation().y) -
        (highRangePoint_obs.translation().x * lowRangePoint_obs.translation().y);

    // Variable that determines if the lidar laser hit an obsticle
    delta = (pow(curR, 2) * pow(dr, 2)) - pow(D, 2);
    
    // Checking if a laser detected an obsticle
    // Checking if the laser is in the same direction as the obsticle
    // Tso = Tws.inv() * Two
    if (delta >= 0.0 && isDetectedPointClose(curBaseScanFrame2D.inv() * obsticleTransforms.at(i), highRangePoint_scan)){
        // Obsticle has been detected, add the dist to the object in m
        if (dy < 0){
            sgn = -1.0;
        } else {
            sgn = 1.0;
        }

        // The points of intersection are given by the two equations below
        // Solve both and find which x and which y are closer to the robot
        // That point finds the intersection point that the sensor "sees"
        intersect1_obs = {turtlelib::Vector2D{((D * dy) + (sgn * dx * sqrt((pow(curR, 2) * pow(dr, 2)) - pow(D, 2)))) / pow(dr, 2),
                        ((-D * dx) + (abs(dy) * sqrt((pow(curR, 2) * pow(dr, 2)) - pow(D, 2)))) / pow(dr, 2)},
                        0.0};

        intersect2_obs = {turtlelib::Vector2D{((D * dy) - (sgn * dx * sqrt((pow(curR, 2) * pow(dr, 2)) - pow(D, 2)))) / pow(dr, 2),
                        ((-D * dx) - (abs(dy) * sqrt((pow(curR, 2) * pow(dr, 2)) - pow(D, 2)))) / pow(dr, 2)},
                        0.0};

        // Convert the points to world frame
        // Tsi = Tws.inv() * Two * Toi
        intersect1_scan = curBaseScanFrame2D.inv() * obsticleTransforms.at(i) * intersect1_obs;
        intersect2_scan = curBaseScanFrame2D.inv() * obsticleTransforms.at(i) * intersect2_obs;

        // Finding the closer point
        if (turtlelib::magnitude(intersect1_scan.translation()) <= turtlelib::magnitude(intersect2_scan.translation())){
            intersect_scanActual = intersect1_scan;
        }else{
            intersect_scanActual = intersect2_scan;
        }

        // Adding noise to the detected lidar range
        noiseAddedLidarRange = addLidarNoise(turtlelib::magnitude(intersect_scanActual.translation()));

        // Only addinge the point if it's the closest detected obsticle point
        if (laserScanMessage.ranges.at(curLidarVectorIndex) == 0.0){
            laserScanMessage.ranges.at(curLidarVectorIndex) = noiseAddedLidarRange;

        }else if (turtlelib::magnitude(intersect_scanActual.translation()) <
                    laserScanMessage.ranges.at(curLidarVectorIndex)){
            laserScanMessage.ranges.at(curLidarVectorIndex) = noiseAddedLidarRange;
        }
    }
}

// Helper function to add lidar noise to the calculated lidar distance
double addLidarNoise(double lidarDist){
    return lidarDist + (lidarNoise(generator) * get_parameter("robot_params.lidar_noise").as_double());
}

// Helper function to check if the laser is pointing towards the obsticle that it was checked against
bool isDetectedPointClose(turtlelib::Transform2D scanner_obs, turtlelib::Transform2D highRangePoint_scan){
    // Getting the signs of the variables
    if (turtlelib::dot(scanner_obs.translation(), highRangePoint_scan.translation()) > 0){
        return true;
    }

    return false;
}

// Helper function to check for lidar wall colissions
void checkLidarToWalls(){
        // Getting the current scanner frame
        curBaseScanFrame2D = turtlebot.position() * turtlelib::Transform2D{{-0.032, 0.0}, curAngle};

        // Creating points at the low and high ranges of the lidar
        // Tsr
        lowRangePoint_scan = {{get_parameter("robot_params.min_lidar_range").as_double(), 0.0}, 0.0};
        highRangePoint_scan = {{get_parameter("robot_params.max_lidar_range").as_double(), 0.0}, 0.0};

        // Creating the min and max range points in the world frame
        // Twr = Tws * Tsr
        lowRangePoint_world = curBaseScanFrame2D * lowRangePoint_scan;
        highRangePoint_world = curBaseScanFrame2D * highRangePoint_scan;

        curLaserSlope = ((highRangePoint_world.translation().y - lowRangePoint_world.translation().y) /
            (highRangePoint_world.translation().x - lowRangePoint_world.translation().x));

        // Checking x positive wall
        if (highRangePoint_world.translation().x > (get_parameter("arena_x_length").as_double() / 2.0)
            && lowRangePoint_world.translation().x < (get_parameter("arena_x_length").as_double() / 2.0)){

            // Find the point that the laser hit the wall
            wallPointScanVec.x = (get_parameter("arena_x_length").as_double() / 2.0) -
                curBaseScanFrame2D.translation().x;

            wallPointScanVec.y = tan(curAngle + turtlebot.position().rotation()) * wallPointScanVec.x;

            // The laser hit the wall and fell within the laser's range, add the wall point to the laserMessage
            wallLidarHelper(turtlelib::magnitude(wallPointScanVec));
        }
        // Checking y positive wall
        if (highRangePoint_world.translation().y > (get_parameter("arena_y_length").as_double() / 2.0) &&
                lowRangePoint_world.translation().y < (get_parameter("arena_y_length").as_double() / 2.0)){

            // Find the point that the laser hit the wall
            wallPointScanVec.y = (get_parameter("arena_y_length").as_double() / 2.0) -
                curBaseScanFrame2D.translation().y;

            wallPointScanVec.x = wallPointScanVec.y / tan(curAngle + turtlebot.position().rotation());

            // The laser hit the wall and fell within the laser's range, add the wall point to the laserMessage
            wallLidarHelper(turtlelib::magnitude(wallPointScanVec));
        }
        // Checking x negative wall
        if (highRangePoint_world.translation().x < (-get_parameter("arena_x_length").as_double() / 2.0) &&
                 lowRangePoint_world.translation().x > (-get_parameter("arena_x_length").as_double() / 2.0)){

            // Find the point that the laser hit the wall
            wallPointScanVec.x = (-get_parameter("arena_x_length").as_double() / 2.0) -
                curBaseScanFrame2D.translation().x;

            wallPointScanVec.y = tan(curAngle + turtlebot.position().rotation()) * wallPointScanVec.x;

            // The laser hit the wall and fell within the laser's range, add the wall point to the laserMessage
            wallLidarHelper(turtlelib::magnitude(wallPointScanVec));
        }
        // Checking y negative wall
        if (highRangePoint_world.translation().y < (-get_parameter("arena_y_length").as_double() / 2.0) &&
                 lowRangePoint_world.translation().y > (-get_parameter("arena_y_length").as_double() / 2.0)){

            // Find the point that the laser hit the wall
            wallPointScanVec.y = (-get_parameter("arena_y_length").as_double() / 2.0) -
                curBaseScanFrame2D.translation().y;

            wallPointScanVec.x = wallPointScanVec.y / tan(curAngle + turtlebot.position().rotation());

            // The laser hit the wall and fell within the laser's range, add the wall point to the laserMessage
            wallLidarHelper(turtlelib::magnitude(wallPointScanVec));
        }
    }

// Helper function to change the lidar range if it hits a wall
void wallLidarHelper(double wallPointMag){
    // If the current range is 0.0 then it was just cleared
    if (laserScanMessage.ranges.at(curLidarVectorIndex) == 0.0){
        // Add noise and update the range value
        laserScanMessage.ranges.at(curLidarVectorIndex) = addLidarNoise(wallPointMag);
    }
    // If the range calculated is less than the current range, 
    else if (wallPointMag < laserScanMessage.ranges.at(curLidarVectorIndex)){
        // Add noise and update the range value
        laserScanMessage.ranges.at(curLidarVectorIndex) = addLidarNoise(wallPointMag);
    }
}

// Helper function to find current distance to a marker
  double distanceToMarker(visualization_msgs::msg::Marker &marker){
      return sqrt(pow(turtlebot.position().translation().x - marker.pose.position.x, 2) +
                  pow(turtlebot.position().translation().y - marker.pose.position.y, 2));
  }

// Helper function to check if the robot collided with an obstacle
void checkObstacleCollision(){
    // Based on the new position of the robot check if it colided with any obsticles
    for (long unsigned int i = 0; i < get_parameter("obsticles/x").as_double_array().size(); i++) {
        // Checking if the turtlebot's circle intersected with the current obsticle
        if (distanceToMarker(obsticleMarkArray.markers.at(i)) < 
            (get_parameter("robot_params.collision_radius").as_double() + get_parameter("obsticles/r").as_double())){
            // A collision happened, call collision update function
            std::vector<double> newPosition = collisionUpdate(i);

            turtlebot.teleport(newPosition.at(0), newPosition.at(1), turtlebot.position().rotation());

            break; // We can assume that the turtlebot can only collide with one obsticle at a time
        }
    }
    // Even if a collision happened the wheels still move as though the wheels slipped on the ground
    // and the robot didn't move
}

// Helper function to check if the robot collided with any wall
void checkWallCollision(){
    // Check x positive wall
    if (turtlebot.position().translation().x + get_parameter("robot_params.collision_radius").as_double() > 
        get_parameter("arena_x_length").as_double() / 2){
        // Move the robot's x position back to within the bounds
        turtlebot.teleport((get_parameter("arena_x_length").as_double() / 2) -
                              get_parameter("robot_params.collision_radius").as_double(),
                           turtlebot.position().translation().y,
                           turtlebot.position().rotation());
    }
    // Check x negative wall
    if (turtlebot.position().translation().x - get_parameter("robot_params.collision_radius").as_double() < 
        -get_parameter("arena_x_length").as_double() / 2){
        // Move the robot's x position back to within the bounds
        turtlebot.teleport((-get_parameter("arena_x_length").as_double() / 2) +
                               get_parameter("robot_params.collision_radius").as_double(),
                           turtlebot.position().translation().y,
                           turtlebot.position().rotation());
    }
    // Check y positive wall
    if (turtlebot.position().translation().y + get_parameter("robot_params.collision_radius").as_double() > 
        get_parameter("arena_y_length").as_double() / 2){
        // Move the robot's x position back to within the bounds
        turtlebot.teleport(turtlebot.position().translation().x,
                           (get_parameter("arena_y_length").as_double() / 2) -
                               get_parameter("robot_params.collision_radius").as_double(),
                           turtlebot.position().rotation());
    }
    // Check y negative wall
    if (turtlebot.position().translation().y - get_parameter("robot_params.collision_radius").as_double() < 
        -get_parameter("arena_y_length").as_double() / 2){
        // Move the robot's x position back to within the bounds
        turtlebot.teleport(turtlebot.position().translation().x,
                           (-get_parameter("arena_y_length").as_double() / 2) +
                               get_parameter("robot_params.collision_radius").as_double(),
                           turtlebot.position().rotation());
    }
}

// Create arena walls
  void createWalls()
  {
    // Wall parameters
    double wallHeight = 0.25;
    double wallThickness = 0.1;

    // Creating the walls of the arena
    // North wall
    nWall.header.frame_id = "nusim/world";
    nWall.header.stamp = this->get_clock()->now();
    nWall.id = 0;
    nWall.type = visualization_msgs::msg::Marker::CUBE;
    nWall.action = visualization_msgs::msg::Marker::ADD;
    nWall.scale.x = wallThickness;
    nWall.scale.y = get_parameter("arena_y_length").as_double();
    nWall.scale.z = wallHeight;
    nWall.pose.position.x = (get_parameter("arena_x_length").as_double() + wallThickness) / 2;
    nWall.pose.position.y = 0.0;
    nWall.pose.position.z = wallHeight / 2;
    nWall.pose.orientation.x = 0.0;
    nWall.pose.orientation.y = 0.0;
    nWall.pose.orientation.z = 0.0;
    nWall.pose.orientation.w = 0.0;
    nWall.color.r = 1.0;
    nWall.color.g = 0.0;
    nWall.color.b = 0.0;
    nWall.color.a = 1.0;

    wallMarkArray.markers.push_back(nWall);

    // East wall
    eWall.header.frame_id = "nusim/world";
    eWall.header.stamp = this->get_clock()->now();
    eWall.id = 1;
    eWall.type = visualization_msgs::msg::Marker::CUBE;
    eWall.action = visualization_msgs::msg::Marker::ADD;
    eWall.scale.x = get_parameter("arena_x_length").as_double();
    eWall.scale.y = wallThickness;
    eWall.scale.z = wallHeight;
    eWall.pose.position.x = 0.0;
    eWall.pose.position.y = -(get_parameter("arena_y_length").as_double() + wallThickness) / 2;
    eWall.pose.position.z = wallHeight / 2;
    eWall.pose.orientation.x = 0.0;
    eWall.pose.orientation.y = 0.0;
    eWall.pose.orientation.z = 0.0;
    eWall.pose.orientation.w = 0.0;
    eWall.color.r = 1.0;
    eWall.color.g = 0.0;
    eWall.color.b = 0.0;
    eWall.color.a = 1.0;

    wallMarkArray.markers.push_back(eWall);

    // South wall
    sWall.header.frame_id = "nusim/world";
    sWall.header.stamp = this->get_clock()->now();
    sWall.id = 2;
    sWall.type = visualization_msgs::msg::Marker::CUBE;
    sWall.action = visualization_msgs::msg::Marker::ADD;
    sWall.scale.x = wallThickness;
    sWall.scale.y = get_parameter("arena_y_length").as_double();
    sWall.scale.z = wallHeight;
    sWall.pose.position.x = -(get_parameter("arena_x_length").as_double() + wallThickness) / 2;
    sWall.pose.position.y = 0.0;
    sWall.pose.position.z = wallHeight / 2;
    sWall.pose.orientation.x = 0.0;
    sWall.pose.orientation.y = 0.0;
    sWall.pose.orientation.z = 0.0;
    sWall.pose.orientation.w = 0.0;
    sWall.color.r = 1.0;
    sWall.color.g = 0.0;
    sWall.color.b = 0.0;
    sWall.color.a = 1.0;

    wallMarkArray.markers.push_back(sWall);

    // West wall
    wWall.header.frame_id = "nusim/world";
    wWall.header.stamp = this->get_clock()->now();
    wWall.id = 3;
    wWall.type = visualization_msgs::msg::Marker::CUBE;
    wWall.action = visualization_msgs::msg::Marker::ADD;
    wWall.scale.x = get_parameter("arena_x_length").as_double();
    wWall.scale.y = wallThickness;
    wWall.scale.z = wallHeight;
    wWall.pose.position.x = 0.0;
    wWall.pose.position.y = (get_parameter("arena_y_length").as_double() + wallThickness) / 2;
    wWall.pose.position.z = wallHeight / 2;
    wWall.pose.orientation.x = 0.0;
    wWall.pose.orientation.y = 0.0;
    wWall.pose.orientation.z = 0.0;
    wWall.pose.orientation.w = 0.0;
    wWall.color.r = 1.0;
    wWall.color.g = 0.0;
    wWall.color.b = 0.0;
    wWall.color.a = 1.0;

    wallMarkArray.markers.push_back(wWall);

    wallPublisher->publish(wallMarkArray);
  }

// Create obsticles
  void createObsticles()
  {
    // Object parameters
    double obsticleR = get_parameter("obsticles/r").as_double();
    double obsticleH = 0.25;
    std::vector<double> obsticleX = get_parameter("obsticles/x").as_double_array();
    std::vector<double> obsticleY = get_parameter("obsticles/y").as_double_array();
    visualization_msgs::msg::Marker obsticle;

    // Obsticle loop
    for (long unsigned int i = 0; i < obsticleX.size(); i++) {
      obsticle.header.frame_id = "nusim/world";
      obsticle.header.stamp = this->get_clock()->now();
      obsticle.id = i;
      obsticle.type = visualization_msgs::msg::Marker::CYLINDER;
      obsticle.action = visualization_msgs::msg::Marker::ADD;
      obsticle.scale.x = obsticleR * 2;
      obsticle.scale.y = obsticleR * 2;
      obsticle.scale.z = obsticleH;
      obsticle.pose.position.x = obsticleX.at(i);
      obsticle.pose.position.y = obsticleY.at(i);
      obsticle.pose.position.z = obsticleH / 2;
      obsticle.pose.orientation.x = 0.0;
      obsticle.pose.orientation.y = 0.0;
      obsticle.pose.orientation.z = 0.0;
      obsticle.pose.orientation.w = 0.0;
      obsticle.color.r = 1.0;
      obsticle.color.g = 0.0;
      obsticle.color.b = 0.0;
      obsticle.color.a = 1.0;

      obsticleMarkArray.markers.push_back(obsticle);

      // Creating the transforms
      obsticleTransforms.push_back(turtlelib::Transform2D{turtlelib::Vector2D{obsticleX.at(i), obsticleY.at(i)}});
    }

    obsticlePublisher->publish(obsticleMarkArray);
  }

// Helper function to give a new position given the robot collided with an object
  std::vector<double> collisionUpdate(int obsticleIndex){
    double magnitude = distanceToMarker(obsticleMarkArray.markers.at(obsticleIndex));

    std::vector<double> unitVector{(obsticleTransforms.at(obsticleIndex).translation().x -
                                        turtlebot.position().translation().x) / magnitude,
                                   (obsticleTransforms.at(obsticleIndex).translation().y -
                                        turtlebot.position().translation().y) / magnitude};

    double combinedRadius = get_parameter("robot_params.collision_radius").as_double() +
                                get_parameter("obsticles/r").as_double();

    // Scale vector to be of dist (turtlebotRad + obsticleRad) and return that vector
    return std::vector<double>{-(unitVector.at(0) * combinedRadius) +
                                    obsticleMarkArray.markers.at(obsticleIndex).pose.position.x,
                               -(unitVector.at(1) * combinedRadius) +
                                    obsticleMarkArray.markers.at(obsticleIndex).pose.position.y};
  }

// *******************Subscribers******************* //
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr redWheelCmdSubscriber;
  void redWheelCmdCallback(const nuturtlebot_msgs::msg::WheelCommands msg)
  {
      // Update the wheel velocities and convert from mcu to rad/s
      wheelVelocities.at(0) = msg.left_velocity *
          get_parameter("robot_params.motor_cmd_per_rad_sec").as_double();
      wheelVelocities.at(1) = msg.right_velocity *
          get_parameter("robot_params.motor_cmd_per_rad_sec").as_double();

      // Only add noise if the robot isn't stopped
      if (wheelVelocities.at(0) != 0.0 && wheelVelocities.at(1) != 0.0){
        // Adding noise to the wheel velocities
        wheelVelocities.at(0) += wheelNoiseDist(generator) * get_parameter("input_noise").as_double();
        wheelVelocities.at(1) += wheelNoiseDist(generator) * get_parameter("input_noise").as_double();

        // Adding slipping
        wheelVelocities.at(0) *= (1 + (slipDist(generator) * get_parameter("slip_fraction").as_double())); 
        wheelVelocities.at(1) *= (1 + (slipDist(generator) * get_parameter("slip_fraction").as_double()));
      }
  }

// *******************Publishers******************* //
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestepPublisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wallPublisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obsticlePublisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fakeSensorPublisher;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_dataPublisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserScanPublisher;

// *******************Broadcasters******************* //
  std::unique_ptr<tf2_ros::TransformBroadcaster> world_red;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
