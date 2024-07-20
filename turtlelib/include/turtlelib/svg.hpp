#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.

#include <string>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib{
    /// \brief a class that writes to an svg file to draw 2D objects
    class Svg{
        // Creating the starting string for the svg file
        std::string fileString;

        public:
            /// \brief Create an identity transformation
            Svg(){
                fileString = 
                    std::string("<svg width=\"8.500000in\" height=\"11.000000in\" ") +
                    std::string("viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n\n");

                std::string arrowMarker = std::string("<defs>\n\t <marker style=\"overflow:visible\" id=\"Arrow1Sstart\" ") +
                    std::string("refX=\"0.0\" refY=\"0.0\" orient=\"auto\">\n\t\t") +
                    std::string("<path transform=\"scale(0.2) translate(6,0)\" ") +
                    std::string("style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\" ") +
                    std::string("d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"/>\n\t") +
                    std::string("</marker>\n</defs>\n\n");

                fileString += arrowMarker; 
            }

            /// \brief Create a vector in the svg file
            /// \param v the vector to print into the svg
            /// \param color the color the vector should be
            /// \return an int to indicate success status
            int operator()(Vector2D v, std::string color);

            /// \brief Create a vector in the svg file starting at a specified origin
            /// \param v the vector to print into the svg
            /// \param tf the transform the vector is based in
            /// \param color the color the point should be
            /// \return an int to indicate success status
            int operator()(Vector2D v, Transform2D tf, std::string color);

            /// \brief Create a point in the svg file
            /// \param p the point to print into the svg
            /// \param color the color the point should be
            /// \return an int to indicate success status
            int operator()(Point2D p, std::string color);

            /// \brief Create a frame in the svg file
            /// \param tf the Transform2D to print into the svg
            /// \param name the name of the tf to be listed in the svg file
            /// \return an int to indicate success status
            int operator()(Transform2D tf, std::string name);

            /// \brief Finish writing tags in the svg file and save it to a file
            /// \return an int to indicate success status
            int writeToFile(std::string fileName);
    };
};

#endif