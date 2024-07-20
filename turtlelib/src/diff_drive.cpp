#include <iosfwd> // contains forward definitions for iostream objects
#include <vector>
#include <iostream>
#include <stdio.h>

#include "turtlelib/diff_drive.hpp"

using namespace std;
namespace turtlelib{
    //brief Move the robot to a new position based on new wheel rotations
    //param phiLprime new left wheel position
    //param phiRprime new right wheel position
    //returns the SPACE twist based on forward kinematics
    Twist2D DiffDrive::forwardKinematics(double phiLprime, double phiRprime){
        // These equations are based on equation [1] in doc/Kinematics.pdf
        double omegaIn = -((wheelRadius / wheelTrack) * (phiLprime)) + ((wheelRadius / wheelTrack) * (phiRprime));
        double xIn = ((wheelRadius/2) * (phiLprime)) + ((wheelRadius/2) * (phiRprime));
        double yIn = 0.0;

        // Twist based on wheel velocities
        Twist2D tw{omegaIn, xIn, yIn};

        // Update the DiffDrive's configuration
        q *= integrate_twist(tw);

        // Return the twist that results from the input velocities
        return tw;
    }

    //brief Move the robot to a new position based on new wheel rotations
    //param phiLprime new left wheel position
    //param phiRprime new right wheel position
    //returns the BODY twist based on forward kinematics
    Twist2D DiffDrive::forwardKinematicsBody(double phiLprime, double phiRprime){
        // Get space twist
        Twist2D spaceTwist = forwardKinematics(phiLprime,phiRprime);

        // Body to space transform
        Transform2D Tsb;

        // Finding the transform between space and body
        // Check if the rotation is 0
        if (almost_equal(spaceTwist.omega, 0.0)){
            // Pure translation
            Vector2D transVec{spaceTwist.x, spaceTwist.y};

            Tsb(transVec);
        }
        else{
            // Not pure translation
            // Find the location of the frame at the center of rotation
            double ys = (-spaceTwist.x)/spaceTwist.omega;
            double xs = (spaceTwist.y)/spaceTwist.omega;

            // Perform the rotation at the new frame
            Vector2D vecsb{xs, ys};

            Tsb(vecsb);
        }

        // Converting the space twist to the body twist
        return Tsb(spaceTwist);
    }

    //brief Generate wheel velocities based on a given new twist
    //param desiredTwist the desired twist to move the robot along
    //return a vector containing the <phiLdot, phiRdot>
    vector<double> DiffDrive::inverseKinematics(Twist2D desiredTwist){
        // Error checking the input twist
        if (abs(desiredTwist.y) > 0.0 && abs(desiredTwist.x) == 0.0 && abs(desiredTwist.omega) == 0.0){
            // The robot can't move in the y direction alone
            throw logic_error("The robot can't move in the y direction alone");            
        }

        // These equations are based on equation [1] in doc/Kinematics.pdf
        double phiLdot = (1/(wheelRadius)) * (desiredTwist.x - ((wheelTrack/2) * desiredTwist.omega));
        double phiRdot = (1/(wheelRadius)) * (desiredTwist.x + ((wheelTrack/2) * desiredTwist.omega));

        // Return the required velocities
        return vector<double>{phiLdot, phiRdot};
    }

    /// \brief Move the robot to a new position as though it were picked up and put down without the wheels moving
    /// \param x The desired x location to put the robot
    /// \param y The desired x location to put the robot
    /// \param theta The desired rotation of the robot
    /// \return The new location of the robot as a Transform2D
    Transform2D DiffDrive::teleport(double x, double y, double theta){
        this->q = Transform2D(Vector2D{x, y}, theta);

        return q;
    }
}