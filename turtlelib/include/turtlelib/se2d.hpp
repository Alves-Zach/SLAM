#ifndef TURTLELIB_SE2_INCLUDE_GUARD_HPP
#define TURTLELIB_SE2_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.

#include <iosfwd> // contains forward definitions for iostream objects
#include <vector>

#include "turtlelib/geometry2d.hpp"

namespace turtlelib{
    /// \brief represent a 2-Dimensional twist
    struct Twist2D{
        /// \brief the angular velocity
        double omega = 0.0;

        /// \brief the linear x velocity
        double x = 0.0;

        /// \brief the linear y velocity
        double y = 0.0;
    };

    /// \brief print the Twist2D in the format [w x y]
    /// \param os [in/out] the ostream to write to
    /// \param tw the twist to output
    /// \returns the ostream os  with the twist data inserted
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw);

    /// \brief read the Twist2D in the format [w x y] or as w x y
    /// \param is [in/out] the istream to read from
    /// \param tw [out] the twist read from the stream
    /// \returns the istream is with the twist characters removed
    std::istream & operator>>(std::istream & is, Twist2D & tw);

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D{
        private:
            std::vector<std::vector<double>> T;
            double x, y, w;

        public:
            /// \brief Create an identity transformation
            Transform2D(){
                T = {{1.0, 0.0, 0.0},
                     {0.0, 1.0, 0.0},
                     {0.0, 0.0, 1.0}};

                x = 0;
                y = 0;
                w = 0;
            }

            /// \brief create a transformation that is a pure translation
            /// \param trans - the vector by which to translate
            explicit Transform2D(Vector2D trans){
                T = {{1.0, 0.0, trans.x},
                     {0.0, 1.0, trans.y},
                     {0.0, 0.0,     1.0}};

                x = trans.x;
                y = trans.y;
                w = 0.0;
            }

            /// \brief create a pure rotation
            /// \param radians - angle of the rotation, in radians
            explicit Transform2D(double radians){
                T = {{cos(radians), -1 * sin(radians), 0},
                     {sin(radians),      cos(radians), 0},
                     {           0,                 0, 1}};

                // Checking if you get -0 in a value
                for (int i = 0; i < 2; i++){
                    for (int j = 0; j < 2; j++){
                        if (T[i][j] == -0){
                            T[i][j] = 0;
                        }
                    }
                }

                x = 0.0;
                y = 0.0;
                w = radians;
            }

            /// \brief Create a transformation with a translational and rotational
            /// component
            /// \param trans - the translation
            /// \param radians - the rotation, in radians
            Transform2D(Vector2D trans, double radians){
                T = {{cos(radians), -sin(radians), trans.x},
                     {sin(radians),  cos(radians), trans.y},
                     {           0,             0,       1}};

                // Checking if you get -0 in a value
                for (int i = 0; i < 2; i++){
                    for (int j = 0; j < 2; j++){
                        if (T[i][j] == -0){
                            T[i][j] = 0;
                        }
                    }
                }

                x = trans.x;
                y = trans.y;
                w = radians;
            }

            /// \brief apply a transformation to a 2D Point
            /// \param p the point to transform
            /// \return a point in the new coordinate system
            Point2D operator()(Point2D p) const;
            
            /// \brief apply a transformation to a 2D Vector
            /// \param v - the vector to transform
            /// \return a vector in the new coordinate system
            Vector2D operator()(Vector2D v) const;

            /// \brief apply a transformation to a Twist2D (e.g. using the adjoint)
            /// \param v - the twist to transform
            /// \return a twist in the new coordinate system
            Twist2D operator()(Twist2D v) const;

            /// \brief invert the transformation
            /// \return the inverse transformation.
            Transform2D inv() const;

            /// \brief compose this transform with another and store the result
            /// in this object
            /// \param rhs - the first transform to apply
            /// \return a reference to the newly transformed operator
            Transform2D & operator*=(const Transform2D & rhs);

            /// \brief the translational component of the transform
            /// \return the x,y translation
            Vector2D translation() const;

            /// \brief get the angular displacement of the transform
            /// \return the angular displacement, in radians
            double rotation() const;

            /// \brief \see operator<<(...) (declared outside this class)
            /// for a description
            friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
    };

    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(const Transform2D lhs, const Transform2D & rhs);

    // Added in HW2
    
    /// \brief integrate a twist along one time step
    /// \param tw the twist to integrate
    /// \return the transform corisponding to the twist entered
    Transform2D integrate_twist(const Twist2D tw);
}

#endif
