#include <catch2/catch_all.hpp>
#include <iostream>
#include <sstream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
using namespace std;
using namespace Catch::Matchers;

namespace turtlelib{
    double epsilon = 0.001;

    TEST_CASE("Svg writeToFile() works"){
        Svg mySVG;

        REQUIRE_THAT(mySVG.writeToFile("testSVG"), WithinAbs(1, epsilon));
    }
    TEST_CASE("Svg Point2D works"){
        Svg mySVG;
        Point2D p;
        p.x = 2;
        p.y = 3;

        mySVG(p, "red");

        REQUIRE_THAT(mySVG.writeToFile("testSVG"), WithinAbs(1, epsilon));
    }
    TEST_CASE("Svg Vector2D works"){
        Svg mySVG;
        Vector2D v;
        v.x = 2;
        v.y = 3;
        Point2D p;
        p.x = 4;
        p.y = 6;

        mySVG(p, "purple");
        mySVG(v, "purple");

        REQUIRE_THAT(mySVG.writeToFile("testSVG"), WithinAbs(1, epsilon));
    }
    TEST_CASE("Svg Tranform2D works"){
        Svg mySVG;
        Vector2D v1;
        v1.x = 2;
        v1.y = 3;
        Vector2D v2;
        v2.x = 0;
        v2.y = -1;
        Vector2D v3;
        v3.x = 0;
        v3.y = 2;
        Point2D p;
        p.x = 2;
        p.y = 3.5;

        Transform2D world;
        Transform2D tf1(v1, PI/4);
        Transform2D tf2(v3, PI);
        Transform2D tf3(v2, -PI/2);

        mySVG(p, "orange");
        mySVG(v1, "orange");
        mySVG(world, "world");
        mySVG(tf1, "1");
        mySVG(tf2, "2");
        mySVG(tf3, "3");

        REQUIRE_THAT(mySVG.writeToFile("testSVG"), WithinAbs(1, epsilon));
    }
}