#include <fstream>
#include <iostream>
#include "turtlelib/svg.hpp"
using namespace std;

namespace turtlelib{
    //brief Create a vector in the svg file
    //param v the vector to print into the svg
    //return an int to indicate success status
    int Svg::operator()(Vector2D v, string color){
        double svgx, svgy; // The origin of the tf in svg units
        svgx = (96 * v.x) + 408.0;
        svgy = -(96 * v.y) + 528.0;

        string pointString = "<line x1=\"" + to_string(svgx) + 
                            "\" x2=\"408.000000\" y1=\"" + to_string(svgy) +
                            "\" y2=\"528.000000\" stroke=\"" + color + "\" " +
                            "stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" " +
                            "/> /&gt;\n\n";

        fileString += pointString;
    
        return 1;
    }

    //brief Create a vector in the svg file starting at a specified origin
    //param v the vector to print into the svg
    //return an int to indicate success status
    int Svg::operator()(Vector2D v, Transform2D tf, string color){
        double svgxo, svgxh, svgyo, svgyh; // The origin of the tf in svg units
        svgxo = (96 * tf.translation().x) + 408.0;
        svgyo = -(96 * tf.translation().y) + 528.0;

        // Creating a new vector to the final position in the desired frame
        Vector2D vecH = tf(v);
        svgxh = svgxo + (96 * vecH.x);
        svgyh = svgyo + (-96 * vecH.y);

        string pointString = "<line x1=\"" + to_string(svgxh) + 
                            "\" x2=\"" + to_string(svgxo) + 
                            "\" y1=\"" + to_string(svgyh) +
                            "\" y2=\"" + to_string(svgyo) +
                            "\" stroke=\"" + color + "\" " +
                            "stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" " +
                            "/> /&gt;\n\n";

        fileString += pointString;
    
        return 1;
    }

    //brief Create a point in the svg file
    //param p the point to print into the svg
    //return an int to indicate success status
    int Svg::operator()(Point2D p, string color){
        double svgx, svgy;
        svgx = (96 * p.x) + 408.0;
        svgy = -(96 * p.y) + 528.0;

        string vecString = "<circle cx=\"" + 
                           to_string(svgx) +
                           "\" cy=\"" +
                           to_string(svgy) +
                           "\" r=\"3\" stroke=\"" +
                           color + "\" fill=\"" + color + "\" " +
                           "stroke-width=\"1\"/>\n\n";

        fileString += vecString;

        return 1;
    }

    //brief Create a frame in the svg file
    //param tf the Transform2D to print into the svg
    //return an int to indicate success status
    int Svg::operator()(Transform2D tf, string name){
        double svgx, svgy; // The origin of the tf in svg units
        svgx = 96*tf.translation().x + 408.0;
        svgy = -96*tf.translation().y + 528.0;

        string tfString = "<g>\n\t<line x1=\"" + to_string(svgx + 96*cos(tf.rotation())) +
                          "\" x2=\"" + to_string(svgx) +
                          "\" y1=\"" + to_string(svgy - 96*sin(tf.rotation())) +
                          "\" y2=\"" + to_string(svgy) +
                          "\" stroke=\"red\" stroke-width=\"5\" " +
                          "marker-start=\"url(#Arrow1Sstart)\"/> /&gt;\n\t" +
                          "<line x1=\"" + to_string(svgx + 96*cos(tf.rotation() + PI/2)) +
                          "\" x2=\"" + to_string(svgx) +
                          "\" y1=\"" + to_string(svgy - 96*sin(tf.rotation() + PI/2)) +
                          "\" y2=\"" + to_string(svgy) +
                          "\" stroke=\"green\" stroke-width=\"5\" " +
                          "marker-start=\"url(#Arrow1Sstart)\"/> /&gt;" +
                          "\n\t<text x=\"" + to_string(svgx) +
                          "\" y=\"" + to_string(svgy + 0.25) +
                          "\">{" + name + "}</text>\n</g>\n\n";

        fileString += tfString;

        return 1;
    }

    //brief Finish writing tags in the svg file and save it to a file
    //return an int to indicate success status
    int Svg::writeToFile(string fileName){
        ofstream myfile(fileName + ".svg");

        // Writing the file string to the output file
        myfile << fileString;

        // Writing the </svg> tag
        myfile << "</svg>";

        myfile.close();
        
        return 1;
    }
}