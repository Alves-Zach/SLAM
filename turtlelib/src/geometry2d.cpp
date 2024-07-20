#include <cmath>
#include <iostream>
#include <stdio.h>
#include <ctype.h>
#include "turtlelib/geometry2d.hpp"
using namespace std;

namespace turtlelib{
    //brief wrap an angle to (-PI, PI]
    //param rad (angle in radians)
    //return an angle equivalent to rad but in the range (-PI, PI]
    double normalize_angle(double rad){
        // Figuring out if rad is negative or positive
        double coeff = rad/abs(rad);

        while (abs(rad) >= 2 * PI){
            rad = rad - (coeff * 2 * PI);
        }

        while (abs(rad) > PI){
            rad = rad - (coeff * PI);
        }

        // Edge case for the angle being -PI exactly
        if (rad == -PI){
            return PI;
        }

        return rad;
    }

    // Point operators

    //brief output a 2 dimensional point as [xcomponent ycomponent]
    //param os - stream to output to
    //param p - the point to print
    std::ostream & operator<<(std::ostream & os, const Point2D & p){
        os << '[' << p.x << ' ' << p.y << ']';
        return os;
    }

    //brief input a 2 dimensional point
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    //param is - stream from which to read
    //param p [out] - output vector
    /// HINT: See operator>> for Vector2D
    std::istream & operator>>(std::istream & is, Point2D & p){
        // Check the stream to make surer the user entered the point correctly
        if (is.peek() == '['){
            // Entered using [x y]
            // Getting the [
            is.get();

            // Getting x
            is >> p.x;

            // Getting ' '
            is.get();

            // Getting the y
            is >> p.y;

            return is;

        }
        else if (isdigit(is.peek()) == 1){
            // Entered using x y
            // Getting x
            is >> p.x;

            // Getting ' '
            is.get();

            // Getting the y
            is >> p.y;

            return is;
        }
        else{
            // Throw error into the stream
            cout.setstate(ios::failbit);
        }
        
        return is;
    }

    // Vector operators

    //brief Creating a normalized vector
    //return a vector that points in the same direction with magnitude 1
    Vector2D normalize(Vector2D v){
        Vector2D v_hat;
        v_hat.x = v.x/sqrt(pow(v.x, 2) + pow(v.y, 2));
        v_hat.y = v.y/sqrt(pow(v.x, 2) + pow(v.y, 2));

        return v_hat;
    }

    //brief Subtracting one point from another yields a vector
    //param head point corresponding to the head of the vector
    //param tail point corresponding to the tail of the vector
    //return a vector that points from p1 to p2
    /// NOTE: this is not implemented in terms of -= because subtracting two Point2D yields a Vector2D
    Vector2D operator-(const Point2D & head, const Point2D & tail){
        Vector2D vec;

        vec.x = head.x - tail.x;
        vec.y = head.y - tail.y;

        return vec;
    }

    //brief Adding a vector to a point yields a new point displaced by the vector
    //param tail The origin of the vector's tail
    //param disp The displacement vector
    //return the point reached by displacing by disp from tail
    /// NOTE: this is not implemented in terms of += because of the different types
    Point2D operator+(const Point2D & tail, const Vector2D & disp){
        Point2D retPoint;

        retPoint.x = tail.x + disp.x;
        retPoint.y = tail.y + disp.y;

        return retPoint;
    }

    //brief output a 2 dimensional vector as [xcomponent ycomponent]
    //param os - stream to output to
    //param v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << '[' << v.x << ' ' << v.y << ']';
        return os;        
    }

    //brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    //param is - stream from which to read
    //param v [out] - output vector
    std::istream & operator>>(std::istream & is, Vector2D & v){
        // Check the stream to make surer the user entered the point correctly
        if (is.peek() == '['){
            // Entered using [x y]
            // Getting the [
            is.get();

            // Getting x
            is >> v.x;

            // Getting ' '
            is.get();

            // Getting the y
            is >> v.y;

            return is;

        }
        else if (isdigit(is.peek()) == 1){
            // Entered using x y
            // Getting x
            is >> v.x;

            // Getting ' '
            is.get();

            // Getting the y
            is >> v.y;

            return is;
        }
        else{
            // Throw error into the stream
            cout.setstate(ios::failbit);
        }
        
        return is;
    }

    // Added as part of HW 2

    //brief Adds a vector to the current vector
    //param rhs vector to be added to the current vector
    //return a reference to the modified vector
    Vector2D & Vector2D::operator+=(const Vector2D & rhs){
        x += rhs.x;
        y += rhs.y;
        
        return *this;
    }

    //brief Adds a vector to the current vector
    //param rhs vector to be added to the current vector
    //return a reference to the modified vector
    Vector2D operator+(const Vector2D & lhs, const Vector2D & rhs){
        Vector2D newVec = lhs;
        newVec += rhs;
        
        return newVec;
    }

    //brief Subtracts a vector to the current vector
    //param rhs vector to be added to the current vector
    //return a reference to the modified vector
    Vector2D & Vector2D::operator-=(const Vector2D & rhs){
        x -= rhs.x;
        y -= rhs.y;
        
        return *this;
    }
    
    //brief Subtracts a vector from the current vector
    //param rhs vector to be added to the current vector
    //return a reference to the modified vector
    Vector2D operator-(const Vector2D & lhs, const Vector2D & rhs){
        Vector2D newVec = lhs;
        newVec -= rhs;
        
        return newVec;
    }

    //brief Multiplies a vector by a double
    //param rhs vector to be added to the current vector
    //return a reference to the modified vector
    Vector2D & Vector2D::operator*=(const double rhs){
        x *= rhs;
        y *= rhs;
        
        return *this;
    }

    //brief Multiplies a vector by a double
    //param rhs vector to be added to the current vector
    //return a reference to the modified vector
    Vector2D operator*(const Vector2D & lhs, const double rhs){
        Vector2D newVec = lhs;
        newVec *= rhs;

        return newVec;
    }

    //brief Multiplies a vector by the current vector
    //param rhs vector to be added to the current vector
    //return a reference to the modified vector
    Vector2D operator*(const double lhs, const Vector2D & rhs){
        Vector2D newVec = rhs;
        newVec *= lhs;

        return newVec;
    }

    //brief Multiplies a vector by the current vector
    //param rhs vector to be added to the current vector
    //return a reference to the modified vector
    double dot(const Vector2D & v1, const Vector2D & v2){
        return (v1.x*v2.x)+(v1.y*v2.y);
    }

    //brief Multiplies a vector by the current vector
    //param rhs vector to be added to the current vector
    //return a reference to the modified vector
    double magnitude(const Vector2D & v1){
        return sqrt(pow(v1.x, 2) + pow(v1.y, 2));
    }

    //brief Finds the angle between two vectors
    //param v1 vector to be added to the current vector
    //return a reference to the modified vector
    double angle(const Vector2D & v1, const Vector2D & v2){
        // Gets angle going in positive rotation direction
        double posAngle = atan2(v2.y, v2.x)- atan2(v1.y, v1.x);

        // Checking if the angle is greater than PI
        if (posAngle > PI){
            return posAngle - PI;
        }
        if (posAngle < -PI){
            return -(posAngle + PI);
        }
        else {
            return posAngle;
        }
    }
}