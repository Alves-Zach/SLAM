#ifndef TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include <iosfwd> // contains forward definitions for iostream objects
#include <vector>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib{
    /// @brief position and orientation control for a differential drive robot
    class DiffDrive{
        private:
            // Member variables
            double wheelRadius, wheelTrack;
            double phiL = 0, phiR = 0; // Starting wheel rotations are assumed to be 0
            Transform2D q; // The robot's position relative to it's starting position

        public:
            // Constructors
            
            /// @brief Constructor assuming starting twist of (0, 0, 0)
            /// @param wheelTrackIn input wheel track distance (m)
            /// @param wheelRadiusIn input wheel radius (m)
            explicit DiffDrive(double wheelRadiusIn = 0.033, double wheelTrackIn = 0.16){
                wheelTrack = wheelTrackIn;
                wheelRadius = wheelRadiusIn;
            }

            /// @brief Constructor setting the starting position with a Transform2D
            /// @param startingConfig Transform2D of where the robot starts
            /// @param wheelTrackIn input wheel track distance (m)
            /// @param wheelRadiusIn input wheel radius (m)
            explicit DiffDrive(Transform2D startingConfig, double wheelRadiusIn = 0.033,
                            double wheelTrackIn = 0.16){
                q = startingConfig;
                wheelTrack = wheelTrackIn;
                wheelRadius = wheelRadiusIn;
            }

            // Getters
            
            /// @brief getter for the current position of the robot
            /// @return the current position of the robot as a Transform2D
            Transform2D position(){return q;}

            /// @brief getter for the wheel radius of the robot
            /// @return the radius of the robot's wheels as a double
            double getWheelRadius(){return wheelRadius;}

            /// @brief getter for the track of the robot
            /// @return the robot's track as a double
            double getTrack(){return wheelTrack;}

            // Functions
            
            /// \brief Move the robot to a new position based on new wheel rotations
            /// \param phiLprime new left wheel position
            /// \param phiRprime new right wheel position
            /// \returns the SPACE twist based on forward kinematics
            Twist2D forwardKinematics(double phiLprime, double phiRprime);

            /// \brief Move the robot to a new position based on new wheel rotations
            /// \param phiLprime new left wheel position
            /// \param phiRprime new right wheel position
            /// \returns the BODY twist based on forward kinematics
            Twist2D forwardKinematicsBody(double phiLprime, double phiRprime);

            /// \brief Generate wheel velocities based on a given new twist
            /// \param desiredTwist the desired twist to move the robot along
            /// \return a vector containing the <phiLdot, phiRdot>
            std::vector<double> inverseKinematics(Twist2D desiredTwist);

            /// \brief Move the robot to a new position as though it were picked up and put down
            /// \param x The desired x location to put the robot
            /// \param y The desired x location to put the robot
            /// \param theta The desired rotation of the robot
            /// \return The new location of the robot as a Transform2D
            Transform2D teleport(double x, double y, double theta);
        };
}

#endif