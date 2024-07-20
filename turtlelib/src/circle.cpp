#include <vector>
#include <numeric>

#include "turtlelib/circle.hpp"

using namespace std;
namespace turtlelib{
    // Circle fitting function based on Nikolai Chernov's Circle Fitting Examples
    // https://people.cas.uab.edu/~mosya/cl/CPPcircle.html
    Circle fit(const vector<double> xPoints, const vector<double> yPoints){
        // Find the number of points in the input
        int numPoints = xPoints.size();

        // Find the means of the x and y points
        double meanX = averageVector(xPoints);
        double meanY = averageVector(yPoints);

        // Computing moments based on given points
        double Mxx, Myy, Mxy, Mxz, Myz, Mzz;
        Mxx = Myy = Mxy = Mxz = Myz = Mzz = 0.0;
        for(int i = 0; i < numPoints; i++){
            double Xi = xPoints[i] - meanX;
            double Yi = yPoints[i] - meanY;
            double Zi = Xi*Xi + Yi*Yi;

            Mxy += Xi*Yi;
            Mxx += Xi*Xi;
            Myy += Yi*Yi;
            Mxz += Xi*Zi;
            Myz += Yi*Zi;
            Mzz += Zi*Zi;
        }

        // Normalizing the moments
        Mxx /= numPoints;
        Myy /= numPoints;
        Mxy /= numPoints;
        Mxz /= numPoints;
        Myz /= numPoints;
        Mzz /= numPoints;

        // Computing coefficients of the polynomial
        double Mz = Mxx + Myy;
        double Cov_xy = Mxx*Myy - Mxy*Mxy;
        double Var_z = Mzz - Mz*Mz;

        double A2 = 4*Cov_xy - 3*Mz*Mz - Mzz;
        double A1 = Var_z*Mz + 4*Cov_xy*Mz - Mxz*Mxz - Myz*Myz;
        double A0 = Mxz*(Mxz*Myy - Myz*Mxy) + Myz*(Myz*Mxx - Mxz*Mxy) - Var_z*Cov_xy;
        double A22 = A2 + A2;

        // Using Newton's method to find the root of the polynomial
        // Variables to use in the foor loop
        double x = 0.0, y = A0;
        double Dy, xnew, ynew;
        int iteration;
        for (iteration = 0; iteration < 99; iteration++){
            Dy = A1 + (x * (A22 + (16.0 * (x * x))));
            xnew = x - (y / Dy);

            // If the x value of the new x spot hasn't changed, a good value was found
            if (x == xnew || !isfinite(xnew)){
                break;
            }

            // Find the new ynew value
            ynew = A0 + (xnew * (A1 + (xnew * (A2 + (4 * xnew * xnew)))));

            // If the ynew is greater than the y, the x value is good
            if (ynew > y){
                break;
            }

            // If end conditions are not met, update the x and y values
            x = xnew;
            y = ynew;
        }

        // Creating the circle to output
        Circle circle;

        // Find the circle's parameters
        double DET = x*x - x*Mz + Cov_xy;
        double Xcenter = ((Mxz*(Myy - x) - x) / DET / 2.0);
        double Ycenter = ((Myz*(Mxx - x) - y) / DET / 2.0);

        // Assign the circle's parameters
        circle.x = Xcenter + meanX;
        circle.y = Ycenter + meanY;
        circle.radius = sqrt(Xcenter*Xcenter + Ycenter*Ycenter + (Mz - x - x));
        circle.sigmaEst = sqrt(y / numPoints + (x*x) / (DET * 2.0));
        circle.i = 0;
        circle.j = iteration;

        return circle;
    }

    // Finds the average of a vector
    double averageVector(std::vector<double> vec){
        return accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
    }
}