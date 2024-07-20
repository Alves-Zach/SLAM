#include <cmath>
#include <iostream>
#include <stdio.h>
#include <ctype.h>
#include "turtlelib/se2d.hpp"
using namespace std;

namespace turtlelib{
    //brief print the Twist2D in the format [w x y]
    //param os [in/out] the ostream to write to
    //param tw the twist to output
    //returns the ostream os  with the twist data inserted
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw){
        os << '[' << tw.omega << ' ' << tw.x << ' ' << tw.y << ']';
        return os;
    }

    //brief read the Twist2D in the format [w x y] or as w x y
    //param is [in/out] the istream to read from
    //param tw [out] the twist read from the stream
    //returns the istream is with the twist characters removed
    std::istream & operator>>(std::istream & is, Twist2D & tw){
        double win;
        // Check the stream to make surer the user entered the point correctly
        if (is.peek() == '['){
            // Entered using [x y]
            // Getting the [
            is.get();

            // Getting w
            is >> win;

            // Getting ' '
            is.get();

            // Getting the x
            is >> tw.x;

            // Getting ' '
            is.get();

            // Getting the y
            is >> tw.y;

            // Setting the angle
            tw.omega = win;

            return is;
        }
        else if (isdigit(is.peek()) == 1 || is.peek() == '-'){
            double win;
            // Entered using w x y
            // Getting w
            is >> win;

            // Getting ' '
            is.get();

            // Getting the x
            is >> tw.x;

            // Getting ' '
            is.get();

            // Getting the y
            is >> tw.y;

            // Setting the angle
            tw.omega = win;

            return is;
        }
        else{
            // Throw error into the stream
            cout.setstate(ios::failbit);
        }
        
        return is;
    }

    //brief apply a transformation to a 2D Point
    //param p the point to transform
    //return a point in the new coordinate system
    Point2D Transform2D::operator()(Point2D p) const{
        turtlelib::Point2D returnPoint;
        returnPoint.x = p.x * T.at(0).at(0) + p.y * T.at(0).at(1) + T.at(0).at(2);
        returnPoint.y = p.x * T.at(1).at(0) + p.y * T.at(1).at(1) + T.at(1).at(2);

        return returnPoint;
    }
    
    //brief apply a transformation to a 2D Vector
    //param v - the vector to transform
    //return a vector in the new coordinate system
    Vector2D Transform2D::operator()(Vector2D v) const{
        turtlelib::Vector2D returnVect;
        returnVect.x = v.x * T.at(0).at(0) + v.y * T.at(0).at(1);
        returnVect.y = v.x * T.at(1).at(0) + v.y * T.at(1).at(1);

        return returnVect;
    }

    //brief apply a transformation to a Twist2D (e.g. using the adjoint)
    //param v - the twist to transform
    //return a twist in the new coordinate system
    Twist2D Transform2D::operator()(Twist2D v) const{
        Twist2D newTwist;

        newTwist.omega = v.omega;
        newTwist.x = (cos(w) * v.x) - (sin(w) * v.y) + (y * v.omega);
        newTwist.y = (sin(w) * v.x) + (cos(w) * v.y) - (x * v.omega);

        return newTwist;
    }

    //brief invert the transformation
    //return the inverse transformation.
    Transform2D Transform2D::inv() const{
        // Finding the inverse translation
        Vector2D newVec;
        newVec.x = -T.at(0).at(0) * translation().x - T.at(1).at(0) * translation().y;
        newVec.y = -T.at(0).at(1) * translation().x - T.at(1).at(1) * translation().y;

        double win = -rotation(); 

        // Checking for 0.0 so -0 doesn't happen
        if (rotation() == 0.0){
            win = rotation();
        }

        Transform2D invt(newVec, win);

        return invt;
    }

    //brief compose this transform with another and store the result in this object
    //param rhs - the first transform to apply
    //return a reference to the newly transformed operator
    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        Transform2D newT;

        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                newT.T.at(i).at(j) = 
                T.at(i).at(0)*rhs.T.at(0).at(j) + T.at(i).at(1)*rhs.T.at(1).at(j) + T.at(i).at(2)*rhs.T.at(2).at(j);
            }
        }

        T = newT.T;

        x = T.at(0).at(2);
        y = T.at(1).at(2);
        w = atan2(T.at(1).at(0), T.at(0).at(0));

        return *this;
    }

    //brief the translational component of the transform
    //return the x,y translation
    Vector2D Transform2D::translation() const{
        Vector2D returnVec;
        returnVec.x = x;
        returnVec.y = y;
        
        return returnVec;
    }

    //brief get the angular displacement of the transform
    //return the angular displacement, in radians
    double Transform2D::rotation() const{
        return w;
    }

    //brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    //param os - an output stream
    //param tf - the transform to print
    ostream & operator<<(ostream & os, const Transform2D & tf){
        os << "deg: " << rad2deg(tf.w) << ' '
        << "x: " << tf.x << ' '
        << "y: " << tf.y;
        return os;
    }

    //brief Read a transformation from stdin
    //Should be able to read input either as output by operator<< or
    //as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    istream & operator>>(istream & is, Transform2D & tf){
        double win, xin, yin;
        // Check the stream to make sure the user entered the point correctly
        if (is.peek() == '['){
            // Entered using [x y]
            // Getting the [
            is.get();

            // Getting w
            is >> win;

            // Getting ' '
            is.get();

            // Getting the x
            is >> xin;

            // Getting ' '
            is.get();

            // Getting the y
            is >> yin;

            // Putting those values into a Transform2D
            Vector2D inputVec;
            inputVec.x = xin;
            inputVec.y = yin;

            Transform2D inputTF(inputVec, deg2rad(win));

            tf = inputTF;

            return is;
        }
        else if (isdigit(is.peek()) == 1 || is.peek() == '-'){
            // Entered using x y
            // Getting w
            is >> win;

            // Getting ' '
            is.get();

            // Getting the x
            is >> xin;

            // Getting ' '
            is.get();

            // Getting the y
            is >> yin;

            // Putting those values into a Transform2D
            Vector2D inputVec;
            inputVec.x = xin;
            inputVec.y = yin;

            Transform2D inputTF(inputVec, deg2rad(win));

            tf = inputTF;

            return is;
        }
        else{
            // Throw error into the stream
            cout.setstate(ios::failbit);
        }
        
        return is;
    }

    //brief multiply two transforms together, returning their composition
    //param lhs - the left hand operand
    //param rhs - the right hand operand
    //return the composition of the two transforms
    //HINT: This function should be implemented in terms of *=
    Transform2D operator*(const Transform2D lhs, const Transform2D & rhs){
        Transform2D newT(lhs);

        newT *= rhs;

        return newT;        
    }

    // Added in HW2

    //brief integrate a twist along one time step
    //param tw the twist to integrate
    //return the transform corisponding to the twist entered
    Transform2D integrate_twist(const Twist2D tw){
        // Check if the rotation is 0
        if (tw.omega == 0.0){
            // Pure translation
            Vector2D transVec{tw.x, tw.y};

            return Transform2D(transVec);
        }
        else{
            // Not pure translation
            // Find the location of the frame at the center of rotation
            double ys = (-tw.x)/tw.omega;
            double xs = (tw.y)/tw.omega;

            // Perform the rotation at the new frame
            Vector2D vecsb{xs, ys};

            Transform2D Tspbp(vecsb);

            // Tspbp = Tsb, so Tspbp.inv() = Tbs
            Transform2D Tbs = Tspbp.inv();

            // Create Tssp
            Transform2D Tssp(tw.omega);

            // Combining them all into a final transform Tbbp
            return Tbs * Tssp * Tspbp;
        }
    }
}