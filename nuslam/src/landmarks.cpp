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
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/circle.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define PI 3.14159

using namespace std::chrono_literals;

/// \brief Node that controls the slam aspect of this simulation
class Landmarks : public rclcpp::Node{
    public:
        Landmarks() : Node("landmarks"), count_(0) {
            // *******************Parameters******************* //
            rclcpp::QoS markerQoS(10);
            markerQoS.transient_local();

            // *******************Publishers******************* //
            landmarkPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_estimate", markerQoS);

            // *******************Subscribers******************* //
            lidarSubscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("/laser_scan",
                                                                                     10,
                                                                                     std::bind(&Landmarks::lidarCallback,
                                                                                     this, std::placeholders::_1));
        }
    private:
        // *******************Parameters******************* //
        int count_;
        std::vector<std::vector<double>> clustersR, clustersX, clustersY;
        std::vector<double> potentialClusterR, potentialClusterX, potentialClusterY;
        std::vector<double> landmarkXY;
        visualization_msgs::msg::MarkerArray landmarks;
        rclcpp::Time curTime;
        double landmarkX, landmarkY, landmarkRadius;
        int timesThroughlidarCallback = 0;

        // *******************Publishers******************* //
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmarkPublisher;

        // *******************Subscribers******************* //
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidarSubscriber;
        void lidarCallback(const sensor_msgs::msg::LaserScan & msg){
            // Clear the clusters and potential clusters
            clearClustersandPotentialClusters();

            // Go through each point and make clusters
            long unsigned int index = 0;
            for (const float &point : msg.ranges){
                // Only calculate things for points that are on an obstacle
                if (point <= 1e-7){
                    index++;
                    continue;
                }

                // Getting the relative x and y of the point
                landmarkXY = findLandmarkXY(index, point);

                // Check if there are no clusters
                if (clustersR.empty()){
                    potentialClusterR.push_back({msg.ranges.at(index)});
                    potentialClusterX.push_back({landmarkXY.at(0)});
                    potentialClusterY.push_back({landmarkXY.at(1)});

                    // If there are no clusters and the 3rd point is reached, create a new cluster
                    if (potentialClusterR.size() == 3){
                        // Add this and the previous 2 points to the cluster
                        clustersR.push_back(potentialClusterR);
                        clustersX.push_back(potentialClusterX);
                        clustersY.push_back(potentialClusterY);

                        // Clear potential cluster vector
                        potentialClusterR.clear();
                        potentialClusterX.clear();
                        potentialClusterY.clear();
                    }
                }
                
                // If there are clusters, check if the point is close to any of them
                else{
                    bool closeToCluster = false;
                    for (unsigned int i = 0; i < clustersR.size(); i++){
                        // If the point is close enough to a existing cluster, add it to that cluster
                        if (timesThroughlidarCallback == 0){
                            // Use the average values of the clusters to determine if the point is close to the cluster
                            if (distBetweenPoints({turtlelib::averageVector(clustersX.at(i)),
                                                turtlelib::averageVector(clustersY.at(i))},
                                                landmarkXY) < 0.25){
                                clustersR.at(i).push_back(msg.ranges.at(index));
                                clustersX.at(i).push_back(landmarkXY.at(0));
                                clustersY.at(i).push_back(landmarkXY.at(1));

                                closeToCluster = true;
                            }
                        }
                        else{
                            // Use the point of the corrisponding marker
                            if (distBetweenPoints({landmarks.markers.at(i).pose.position.x,
                                                landmarks.markers.at(i).pose.position.y},
                                                landmarkXY) < 0.2){
                                clustersR.at(i).push_back(msg.ranges.at(index));
                                clustersX.at(i).push_back(landmarkXY.at(0));
                                clustersY.at(i).push_back(landmarkXY.at(1));

                                closeToCluster = true;
                            }
                        }
                    }

                    // If not, add to a potential cluster
                    if (!closeToCluster){
                        potentialClusterR.push_back({msg.ranges.at(index)});
                        potentialClusterX.push_back({landmarkXY.at(0)});
                        potentialClusterY.push_back({landmarkXY.at(1)});

                        // If the potential cluster has 3 points, add it to the clusters and clear potential cluster
                        if (potentialClusterR.size() == 3){
                            clustersR.push_back(potentialClusterR);
                            clustersX.push_back(potentialClusterX);
                            clustersY.push_back(potentialClusterY);

                            // Clear potential cluster vector
                            potentialClusterR.clear();
                            potentialClusterX.clear();
                            potentialClusterY.clear();
                        }
                    }
                }

                index++;
            }

            // At the end of each set of points, create a landmark for each cluster
            if (timesThroughlidarCallback == 0){
                for (long unsigned int i = 0; i < clustersR.size(); i++){
                    createLandmark(clustersX.at(i), clustersY.at(i));
                }
            }
            else{
                for (long unsigned int i = 0; i < clustersR.size(); i++){
                    updateLandmark(clustersX.at(i), clustersY.at(i), i);
                }
            }

            // Getting the current time
            curTime = this->now();

            RCLCPP_INFO(this->get_logger(), "Num clusters: %ld", clustersR.size());

            // Publish the landmarks
            landmarkPublisher->publish(landmarks);

            // Incriment times through lidar callback
            timesThroughlidarCallback++;
        }

        // *******************Helper functions******************* //
        // Clear the clusters and potential clusters
        void clearClustersandPotentialClusters(){
            clustersR.clear();
            clustersX.clear();
            clustersY.clear();
            potentialClusterR.clear();
            potentialClusterX.clear();
            potentialClusterY.clear();
        }

        // Clear markers
        void clearMarkers(){
            landmarks.markers.clear();
        }

        // Updates a landmark
        void updateLandmark(std::vector<double> xData, std::vector<double> yData, long unsigned int index){
            RCLCPP_INFO(this->get_logger(), "Updating landmark %ld", index);
            // Create a circle from the data
            turtlelib::Circle circle = turtlelib::fit(xData, yData);
            
            // Update the landmark
            landmarks.markers.at(index).pose.position.x = circle.x;
            landmarks.markers.at(index).pose.position.y = circle.y;
            landmarks.markers.at(index).scale.x = circle.radius * 2;
            landmarks.markers.at(index).scale.y = circle.radius * 2;
        }

        // Creates a landmark from initial conditions
        void createLandmark(std::vector<double> xData, std::vector<double> yData){
            // Create a circle from the data
            turtlelib::Circle circle = turtlelib::fit(xData, yData);

            visualization_msgs::msg::Marker landmark;
            landmark.header.frame_id = "red/base_scan";
            landmark.id = landmarks.markers.size();
            landmark.header.stamp = curTime;
            landmark.type = visualization_msgs::msg::Marker::CYLINDER;
            landmark.action = visualization_msgs::msg::Marker::ADD;
            landmark.pose.position.x = circle.x;
            landmark.pose.position.y = circle.y;
            landmark.pose.position.z = 1.25 * 0.125;
            landmark.pose.orientation.x = 0.0;
            landmark.pose.orientation.y = 0.0;
            landmark.pose.orientation.z = 0.0;
            landmark.pose.orientation.w = 1.0;
            landmark.scale.x = circle.radius * 2;
            landmark.scale.y = circle.radius * 2;
            landmark.scale.z = 1.25 * 0.25;
            landmark.color.a = 1.0;
            landmark.color.r = 1.0;
            landmark.color.g = 1.0;
            landmark.color.b = 28.0/255.0;

            landmarks.markers.push_back(landmark);

            RCLCPP_INFO(this->get_logger(), "Landmark created at x: %f, y: %f, r: %f", circle.x, circle.y, circle.radius);
        }

        // Finds the positions of a point
        std::vector<double> findLandmarkXY(double theta, double range){
            return {range * cos(turtlelib::deg2rad(theta)), range * sin(turtlelib::deg2rad(theta))};
        }

        // Find the distance between two points
        double distBetweenPoints(std::vector<double> point1, std::vector<double> point2){
            return sqrt(pow(point1.at(0) - point2.at(0), 2) + pow(point1.at(1) - point2.at(1), 2));
        }

        // Print a vector
        void printVector(std::vector<double> vec){
            std::ostringstream s;

            s << "\n";

            for (const double &val : vec){
                s << val << ", ";
            }
            
            RCLCPP_INFO(get_logger(), "%s\n", s.str().c_str());
        }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}