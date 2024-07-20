#include <sstream>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <string.h>
#include <vector>
#include <random>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "turtlelib/diff_drive.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#define PI 3.14159

using namespace std::chrono_literals;

/// \brief Node that controls the slam aspect of this simulation
class Slam : public rclcpp::Node{
    public:
        Slam() : Node("slam"), count_(0) {
            // *******************Parameters******************* //
            declare_parameter("body_id", rclcpp::PARAMETER_STRING);
            declare_parameter("odom_id", rclcpp::PARAMETER_STRING);
            declare_parameter("use_lidar", true);
            int numobstacles = declare_parameter("Max_num_obstacles", 20);
            pathDecimation = declare_parameter("path_decimation", 3);

            // QOS object
            rclcpp::QoS markerQoS(10);
            markerQoS.transient_local();

            // Broadcaster for transforms
            tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            // *******************Subscribers******************* //
            // Odometry
            odomSubscriber = this->create_subscription \
            <nav_msgs::msg::Odometry>(
            "/odometry", 10, std::bind(
                &Slam::odomCallback,
                this, std::placeholders::_1));
            // Fake sensor markers
            if (!get_parameter("use_lidar").as_bool()){
                sensorSubscriber = this->create_subscription \
                <visualization_msgs::msg::MarkerArray>(
                "/fake_sensor", markerQoS, std::bind(
                    &Slam::sensorCallback,
                    this, std::placeholders::_1));
            }else{
                sensorSubscriber = this->create_subscription \
                <visualization_msgs::msg::MarkerArray>(
                "/obstacle_estimate", markerQoS, std::bind(
                    &Slam::sensorCallback,
                    this, std::placeholders::_1));
            }

            // *******************Publishers******************* //
            // Path publisher
            pathPublisher = this->create_publisher<nav_msgs::msg::Path>(
            "~/path",
            markerQoS);
            mapPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/map",
            markerQoS);

            // Initializing path object
            robotPath.header.frame_id = "nusim/world";

            // Setting path counter number
            pathCounter = 0;

            // Initializing transforms to publish
            // world to map
            Tworld_mapGeom.header.frame_id = "nusim/world";
            Tworld_mapGeom.child_frame_id = "map";
            Tworld_mapGeom.transform.translation.z = 0.0;
            // map to odom
            Tmap_odomGeom.header.frame_id = "map";
            Tmap_odomGeom.child_frame_id = get_parameter("odom_id").as_string();
            Tmap_odomGeom.transform.translation.z = 0.0;
            // odom to green
            Todom_greenGeom.header.frame_id = get_parameter("odom_id").as_string();
            Todom_greenGeom.child_frame_id = get_parameter("body_id").as_string();
            Todom_greenGeom.transform.translation.z = 0.0;

            // *******************Matricies******************* //
            landmarkInitialized = std::vector<bool>(numobstacles, false);
            ut = arma::mat(3, 1, arma::fill::zeros);
            At = arma::mat(3 + (numobstacles * 2), 3 + (numobstacles * 2), arma::fill::zeros);
            identityAt = arma::eye (3 + (numobstacles * 2), 3 + (numobstacles * 2));
            subAt = arma::mat(3 + (numobstacles * 2), 3 + (numobstacles * 2), arma::fill::zeros);
            del = arma::vec({0, 0});
            Qbar = arma::mat((2 * numobstacles) + 3, (2 * numobstacles) + 3, arma::fill::zeros);
            Sigma_t = arma::join_cols(arma::join_rows(arma::mat(3, 3, arma::fill::zeros),
                                                      arma::mat(3, 2 * numobstacles, arma::fill::zeros)),
                                      arma::join_rows(arma::mat(2 * numobstacles, 3, arma::fill::zeros),
                                                      arma::eye(2 * numobstacles, 2 * numobstacles) * 9e10));
            R = arma::eye(2 * numobstacles, 2 * numobstacles) * 0.1;
            zeta = arma::vec(3 + (2 * numobstacles));
            H = arma::mat(2, 3 + (2 * numobstacles), arma::fill::zeros);
            z = {0.0, 0.0};
            zhat = {0.0, 0.0};

            // Creating Qbar matrix
            Qbar = arma::mat((2 * numobstacles) + 3, (2 * numobstacles) + 3, arma::fill::zeros);
            Qbar(0, 0) = 0.05;
            Qbar(1, 1) = 0.05;
            Qbar(2, 2) = 0.05;

            createMarkers(numobstacles);
        }
    private:
        // *******************Parameters******************* //
        // The node's count
        size_t count_;

        // Estimate of obstacles array
        visualization_msgs::msg::MarkerArray estimatedObsticleArray;

        // Broadcaster
        std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

        // The transforms that this node broadcasts
        geometry_msgs::msg::TransformStamped Tworld_mapGeom;
        turtlelib::Transform2D Tworld_mapTurtle;
        geometry_msgs::msg::TransformStamped Tmap_odomGeom;
        turtlelib::Transform2D Tmap_odomTurtle;
        geometry_msgs::msg::TransformStamped Todom_greenGeom;
        turtlelib::Transform2D Todom_greenTurtle;

        // Transform for world to turtlebot based on odometry
        turtlelib::Transform2D Tworld_odom;

        // Transform for map to robot
        turtlelib::Transform2D Tmap_odom_est;

        // The pose for the green robot's path
        geometry_msgs::msg::PoseStamped internalPoseStamped;
        nav_msgs::msg::Path robotPath;

        // Variable used to decimate path position publishing
        int pathCounter, pathDecimation;

        // *******************Publishers*******************
        // Path publisher
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher;


        // Publisher for the estimated obsticle locations
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mapPublisher;


        //*Odometry subscriber variables*
        // Current timestep
        rclcpp::Time curTime;
        arma::mat ut; // Position vector
        double roll, pitch, theta; // Roll Pitch Yaw
        tf2::Quaternion odomQuaternion; // For converting from quaternion to Euler
        tf2::Matrix3x3 odomMatrix; // For converting from quaternion to Euler
        double deltaX, deltaY, deltaTh; // Delta x, y, theta from this timestep

        // Previous timestep
        double prevTh; // theta from the last timestep
        nav_msgs::msg::Odometry prevOdomMessage; // The previous odometry message
        tf2::Quaternion prevodomQuaternion; // For converting from quaternion to Euler
        tf2::Matrix3x3 prevodomMatrix; // For converting from quaternion to Euler

        // Variables used for math to make reading easier
        arma::mat At;
        arma::mat subAt; // This is the matrix at the second term of equations 9 and 10 depending on deltaTheta
        arma::mat identityAt;

        // *******************Subscribers*******************
        // Odometry
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscriber;
        // Odometry callback to update u matrix
        void odomCallback(const nav_msgs::msg::Odometry & msg){
            // Saving message data into the odometry matrix
            ut(0, 0) = theta;
            ut(1, 0) = msg.pose.pose.position.x;
            ut(2, 0) = msg.pose.pose.position.y;

            // Saving the message into a transform of where the turtlebot is
            Tworld_odom = {{ut(1, 0), ut(2, 0)}, ut(0, 0)};

            // Getting delta X, Y
            deltaX = msg.pose.pose.position.x - prevOdomMessage.pose.pose.position.x;
            deltaY = msg.pose.pose.position.y - prevOdomMessage.pose.pose.position.y;

            // Converting from geometry messages quaternion to rpyyaw
            // Getting Theta
            odomQuaternion.setX(msg.pose.pose.orientation.x);
            odomQuaternion.setY(msg.pose.pose.orientation.y);
            odomQuaternion.setZ(msg.pose.pose.orientation.z);
            odomQuaternion.setW(msg.pose.pose.orientation.w);

            odomMatrix.setRotation(odomQuaternion);
            
            odomMatrix.getRPY(roll, pitch, theta);

            deltaTh = tf2NormalizeAngle(theta - prevTh);

            // Saving the current variables into the previous ones
            prevOdomMessage = msg;
            prevodomQuaternion = odomQuaternion;
            prevodomMatrix = odomMatrix;
            prevTh = theta;       
        }

        // Variables that SLAM uses
        std::vector<bool> landmarkInitialized;
        arma::mat H, subH1, subH2, subH3, subH4;
        arma::vec del;
        double d;
        arma::mat Sigma_t;
        arma::mat Qbar;
        arma::mat R;
        arma::mat Kgain;
        arma::vec z, zhat, zdiff;
        arma::vec zeta;
        double a, b, c;
        double range, bearing;
        
        //Fake sensor
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sensorSubscriber;
        void sensorCallback(const visualization_msgs::msg::MarkerArray & msg){
            // Getting the current time
            curTime = this->get_clock()->now();

            // Publishing the two stationary transforms
            // Publishing the stationary map transform
            world_map_broadcaster(0.0, 0.0, 0.0);

            // Publish the stationary transform from odom to green
            odom_green_broadcaster(0.0, 0.0, 0.0);

            //**************Start of slam implimentation**************//
            // *******Prediction*******
            // Updating the At matrix
            updateAtMatrix();

            // Updating Sigma_t
            Sigma_t = (At * Sigma_t * At.t()) + Qbar;

            // *******Update*******
            // Loop that goes through each obstacle
            for (long unsigned int j = 0; j < msg.markers.size(); j++){
                // Ignoring if the marker is out of range
                if (msg.markers.at(j).action == visualization_msgs::msg::Marker::DELETE){
                    // Setting this landmark to uninitialized
                    landmarkInitialized.at(j) = false;

                    // Hide the obstacle in this node
                    estimatedObsticleArray.markers.at(j).action = visualization_msgs::msg::Marker::DELETE;

                    continue;
                }
                
                // Checking if this landmark has been initialized
                if (!landmarkInitialized.at(j)){
                    // Setting this landmark to initialized
                    landmarkInitialized.at(j) = true;

                    initializeLandmark(j, msg.markers.at(j));
                }

                // Computing the H matrix;
                computeH(j);

                // Computing Kalman gain
                Kgain = (Sigma_t * H.t()) * ((H * Sigma_t * H.t()) +
                                             R.submat(2 * j, 2 * j, (2 * j) + 1, (2 * j) + 1)).i();

                // Computing obstacle measurement z and zhat
                computez(msg.markers.at(j));

                // Computing posterior state update
                zeta += Kgain * zdiff;
                zeta(0) = tf2NormalizeAngle(zeta(0));

                // Computing posterior covariance
                Sigma_t = (identityAt - (Kgain * H)) * Sigma_t;

                // Update estimated markers
                updateMarkers(j);
            }

            // Use the state update to update the world to map transform
            Tmap_odom_est = {{zeta(1), zeta(2)}, zeta(0)};
            Tmap_odomTurtle = Tmap_odom_est * Tworld_odom.inv();

            // Broadcast the new transform from map to odom
            map_odom_broadcaster(Tmap_odom_est.translation().x,
                                 Tmap_odom_est.translation().y,
                                 Tmap_odom_est.rotation());

            if (pathCounter == 0){
                // Publish the green path
                green_pathBroadcaster(Tmap_odom_est.translation().x,
                                    Tmap_odom_est.translation().y,
                                    Tmap_odom_est.rotation());
            }

            pathCounter++;

            if(pathCounter == pathDecimation){
                pathCounter = 0;
            }

            // Publish the green markers
            mapPublisher->publish(estimatedObsticleArray);
        }

        // *******************Helper functions*******************
        // Helper function to update the estimated markers
        void updateMarkers(long unsigned int j){
            estimatedObsticleArray.markers.at(j).pose.position.x = zeta((j * 2) + 3);
            estimatedObsticleArray.markers.at(j).pose.position.y = zeta((j * 2) + 4);

            // Update the stamp for each of the markers
            estimatedObsticleArray.markers.at(j).header.stamp = curTime;
        }

        // Helper function to compute the z matrix (used for readability)
        void computez(visualization_msgs::msg::Marker marker){
            z(0) = sqrt(pow(marker.pose.position.x, 2) + pow(marker.pose.position.y, 2));
            z(1) = atan2(marker.pose.position.y, marker.pose.position.x);
            
            zhat(0) = sqrt(d);
            zhat(1) = tf2NormalizeAngle(atan2(del(1), del(0)) - zeta(0));

            // Finding difference between the measurements and normalizing the difference in angles
            zdiff = z - zhat;
            zdiff(1) = tf2NormalizeAngle(zdiff(1));
        }

        // Helper function to compute the H matrix (used for readability)
        void computeH(long unsigned int j){
            // Clearing H
            H *= 0.0;

            // obstacle del x and y
            del(0) = zeta((2 * j) + 3) - zeta(1);
            del(1) = zeta((2 * j) + 4) - zeta(2);

            // Computing squared distance
            d = pow(del(0), 2) + pow(del(1), 2);

            // Changing the right elements of H
            H(1, 0) = -1.0;
            H(0, 1) = -del(0) / sqrt(d);
            H(1, 1) = del(1) / d;
            H(0, 2) = -del(1) / sqrt(d);
            H(1, 2) = -del(0) / d;
            H(0, (2 * j) + 3) = del(0) / sqrt(d);
            H(1, (2 * j) + 3) = -del(1) / d;
            H(0, (2 * j) + 4) = del(1) / sqrt(d);
            H(1, (2 * j) + 4) = del(0) / d;
        }

        // Helper function to initialize a landmark if it hasn't been added to the model yet
        void initializeLandmark(long unsigned int j, visualization_msgs::msg::Marker marker){
            // Calculating range and bearing based on marker passed
            range = sqrt(pow(marker.pose.position.x, 2) + pow(marker.pose.position.y, 2));
            bearing = atan2(marker.pose.position.y, marker.pose.position.x);

            // Updating zeta
            if (zeta((2 * j) + 3) == 0.0 && zeta((2 * j) + 4) == 0.0){
                zeta((2 * j) + 3) = zeta(1) + (range * cos(bearing + zeta(0)));
                zeta((2 * j) + 4) = zeta(2) + (range * sin(bearing + zeta(0)));
            }

            estimatedObsticleArray.markers.at(j).action = visualization_msgs::msg::Marker::ADD;
            estimatedObsticleArray.markers.at(j).header.stamp = curTime;
            estimatedObsticleArray.markers.at(j).pose.position.x = zeta((2 * j) + 3);
            estimatedObsticleArray.markers.at(j).pose.position.y = zeta((2 * j) + 4);
        }

        void createMarkers(int numMarkers){
            for (int j = 0; j < numMarkers; j++){
                // Creating the marker for the green marker array
                visualization_msgs::msg::Marker dummyMarker;

                double obsticleR = 0.14;
                double obsticleH = 0.25;

                dummyMarker.header.frame_id = "map";
                dummyMarker.id = estimatedObsticleArray.markers.size();
                dummyMarker.type = visualization_msgs::msg::Marker::CYLINDER;
                dummyMarker.action = visualization_msgs::msg::Marker::DELETE;
                dummyMarker.scale.x = obsticleR * 2;
                dummyMarker.scale.y = obsticleR * 2;
                dummyMarker.scale.z = 1.5 * obsticleH;
                dummyMarker.pose.position.z = 1.5 * obsticleH / 2;
                dummyMarker.pose.orientation.x = 0.0;
                dummyMarker.pose.orientation.y = 0.0;
                dummyMarker.pose.orientation.z = 0.0;
                dummyMarker.pose.orientation.w = 0.0;
                dummyMarker.color.r = 0.0;
                dummyMarker.color.g = 1.0;
                dummyMarker.color.b = 0.0;
                dummyMarker.color.a = 0.75;

                estimatedObsticleArray.markers.push_back(dummyMarker);

            }
        }

        // Helper function to update the At matrix
        void updateAtMatrix(){
            // Based on the odometry message
            // Create Twb'
            // Update qt
            // Create At
            if (turtlelib::almost_equal(deltaTh, 0.0)){
                subAt(1, 0) = -1 * deltaX * sin(prevTh);
                subAt(2, 0) = deltaX * cos(prevTh);

                At = identityAt + subAt;
            }
            else{
                a = prevTh + deltaTh;

                b = prevOdomMessage.pose.pose.position.x -
                    ((deltaX / deltaTh) * sin(prevTh)) +
                    ((deltaX / deltaTh) * sin(prevTh + deltaTh));

                c = prevOdomMessage.pose.pose.position.y +
                    ((deltaX / deltaTh) * cos(prevTh)) -
                    ((deltaX / deltaTh) * cos(prevTh + deltaTh));

                subAt(1) = -(c - prevOdomMessage.pose.pose.position.y);
                subAt(2) = b - prevOdomMessage.pose.pose.position.x;

                At = identityAt + subAt;
            }
        }

        // Helpper function to print matricies
        void printMatrix(arma::mat m){
            std::ostringstream s;

            s << "\n";

            for (uint i = 0; i < m.n_rows; i++){
                for (uint j = 0; j < m.n_cols; j++){
                    s << m(i, j) << ",     ";
                }
                s  << "\n";
            }

            RCLCPP_INFO(get_logger(), "%s", s.str().c_str());
        }

        // *******************Broadcaster functions*******************
        geometry_msgs::msg::Quaternion dummyGeomQuaternion; // Dummy variable used to set transform quaternions
        tf2::Quaternion dummyTF2Quaternion; // Dummy variable used to set transform quaternions
        // Helper function to update the path message and publish it
        void green_pathBroadcaster(double x, double y, double theta){
            // Only publishing if the robot moved
            if (!(turtlelib::almost_equal(x, internalPoseStamped.pose.position.x) &&
                  turtlelib::almost_equal(y, internalPoseStamped.pose.position.y))){
                // Updating the internal pose
                internalPoseStamped.pose.position.x = x;
                internalPoseStamped.pose.position.y = y;

                dummyTF2Quaternion.setEuler(0.0, 0.0, theta);
                internalPoseStamped.pose.orientation.set__w(dummyTF2Quaternion.w());
                internalPoseStamped.pose.orientation.set__x(dummyTF2Quaternion.x());
                internalPoseStamped.pose.orientation.set__y(dummyTF2Quaternion.y());
                internalPoseStamped.pose.orientation.set__z(dummyTF2Quaternion.z());

                // Stamping the pose
                internalPoseStamped.header.stamp = curTime;

                // Add a pose to the path message
                this->robotPath.poses.push_back(internalPoseStamped);
                // Send the nav_msgs path message
                robotPath.header.stamp = curTime;
                pathPublisher->publish(this->robotPath);
            }
        }

        // Helper functions to update internal transforms and publish them
        void world_map_broadcaster(double x, double y, double theta){
            // Read message content and assign it to
            // corresponding tf variables
            // Turtle only exists in 2D, thus we get x and y translation
            Tworld_mapGeom.transform.translation.x = x;
            Tworld_mapGeom.transform.translation.y = y;

            dummyTF2Quaternion.setEuler(0.0, 0.0, theta);
            Tworld_mapGeom.transform.rotation.set__w(dummyTF2Quaternion.w());
            Tworld_mapGeom.transform.rotation.set__x(dummyTF2Quaternion.x());
            Tworld_mapGeom.transform.rotation.set__y(dummyTF2Quaternion.y());
            Tworld_mapGeom.transform.rotation.set__z(dummyTF2Quaternion.z());

            Tworld_mapGeom.header.stamp = curTime;

            // Send the transformation
            tfBroadcaster->sendTransform(Tworld_mapGeom);
        }
        void map_odom_broadcaster(double x, double y, double theta){
            // Read message content and assign it to
            // corresponding tf variables
            // Turtle only exists in 2D, thus we get x and y translation
            Tmap_odomGeom.transform.translation.x = x;
            Tmap_odomGeom.transform.translation.y = y;

            dummyTF2Quaternion.setEuler(0.0, 0.0, theta);
            Tmap_odomGeom.transform.rotation.set__w(dummyTF2Quaternion.w());
            Tmap_odomGeom.transform.rotation.set__x(dummyTF2Quaternion.x());
            Tmap_odomGeom.transform.rotation.set__y(dummyTF2Quaternion.y());
            Tmap_odomGeom.transform.rotation.set__z(dummyTF2Quaternion.z());

            Tmap_odomGeom.header.stamp = curTime;

            // Send the transformation
            tfBroadcaster->sendTransform(Tmap_odomGeom);
        }
        void odom_green_broadcaster(double x, double y, double theta){
            // Read message content and assign it to
            // corresponding tf variables
            // Turtle only exists in 2D, thus we get x and y translation
            Todom_greenGeom.transform.translation.x = x;
            Todom_greenGeom.transform.translation.y = y;

            dummyTF2Quaternion.setEuler(0.0, 0.0, theta);
            Todom_greenGeom.transform.rotation.set__w(dummyTF2Quaternion.w());
            Todom_greenGeom.transform.rotation.set__x(dummyTF2Quaternion.x());
            Todom_greenGeom.transform.rotation.set__y(dummyTF2Quaternion.y());
            Todom_greenGeom.transform.rotation.set__z(dummyTF2Quaternion.z());

            Todom_greenGeom.header.stamp = curTime;

            // Send the transformation
            tfBroadcaster->sendTransform(Todom_greenGeom);
        }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}