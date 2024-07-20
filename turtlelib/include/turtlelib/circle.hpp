#ifndef TURTLELIB_CIRCLE_INCLUDE_GUARD_HPP
#define TURTLELIB_CIRCLE_INCLUDE_GUARD_HPP

// Implimentation based on Nikolai Chernov's Circle Fitting Examples
// https://people.cas.uab.edu/~mosya/cl/CPPcircle.html

#include <vector>

#include "turtlelib/geometry2d.hpp"

namespace turtlelib{
    class Circle{
        public:
            // Member variables
            double x, y, radius, sigmaEst, gradNorm, Gx, Gy;
            int i, j;
            double error;

            // Constructors
            // Default constructor
            explicit Circle(){
                x = 0.0;
                y = 0.0;
                radius = 1.0;
                sigmaEst = 0.0;
                i = 0;
                j = 0;
                error = 0.0;
            }

            // Constructor with assignment of the circle parameters only
            explicit Circle(double xIn, double yIn, double radiusIn){
                x = xIn;
                y = yIn;
                radius = radiusIn;
                error = 0.0;
            }
    };

    // Helper functions

    // Finds the average of a vector
    double averageVector(std::vector<double> vec);

    // Circle fitting function based on Nikolai Chernov's Circle Fitting Examples
    // https://people.cas.uab.edu/~mosya/cl/CPPcircle.html
    Circle fit(const std::vector<double> xPoints, const std::vector<double> yPoints);
}

#endif