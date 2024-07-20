#include <catch2/catch_all.hpp>
#include <iostream>
#include <sstream>
#include "turtlelib/geometry2d.hpp"
using namespace std;
using namespace Catch::Matchers;

namespace turtlelib{
    double epsilon = 0.001;

    // Testing helper functions
    TEST_CASE("almost_zero works", "[almost_zero]"){ // Zachary, Alves
        REQUIRE(almost_equal(1, 1) == true);
        REQUIRE(almost_equal(0, 1) == false);
        REQUIRE(almost_equal(1, 0) == false);
        REQUIRE(almost_equal(1, 0, 1.01) == true);
        REQUIRE(almost_equal(1, 9, 10) == true);
    }
    TEST_CASE("deg2rad works", "[deg2rad]"){ // Zachary, Alves
        REQUIRE(deg2rad(0) == 0);
        REQUIRE(deg2rad(180) == PI);
        REQUIRE(deg2rad(90) == PI/2);
        REQUIRE(deg2rad(360) == 2*PI);
        REQUIRE(deg2rad(-180) == -PI);
    }
    TEST_CASE("rad2deg works", "[rad2deg]"){ // Zachary, Alves
        REQUIRE(rad2deg(0) == 0);
        REQUIRE(rad2deg(PI) == 180);
        REQUIRE(rad2deg(PI/2) == 90);
        REQUIRE(rad2deg(-PI/2) == -90);
        REQUIRE(rad2deg(-PI/4) == -45);
    }
    TEST_CASE("normalize_angle works", "[normalize_angle]"){ // Zachary, Alves
        REQUIRE(normalize_angle(0) == 0);
        REQUIRE(normalize_angle(PI) == PI);
        REQUIRE(normalize_angle(5) == 5-PI);
        REQUIRE(normalize_angle(-5) == -5+PI);
        REQUIRE(normalize_angle(-PI) == PI);
        REQUIRE(normalize_angle(-PI/4) == -PI/4);
        REQUIRE(normalize_angle(3*PI/2) == PI/2);
        REQUIRE(normalize_angle(-5*PI/2) == -PI/2);
        REQUIRE(normalize_angle(2*PI) == 0.0);
    }

    // Testing Point2D
    TEST_CASE("Point2D << works"){ // Zachary, Alves
        Point2D testPoint;
        testPoint.x = 1.2;
        testPoint.y = 3.4;

        std::ostringstream strStream;
        strStream << testPoint;

        REQUIRE(strcmp(strStream.str().c_str(), "[1.2 3.4]") == 0);
        REQUIRE(strcmp(strStream.str().c_str(), "[3.0 4.0]") != 0);
    }
    TEST_CASE("Point2D >> works"){ // Zachary, Alves
        Point2D testPoint;
        testPoint.x = 1.2;
        testPoint.y = 3.4;

        stringstream testString;
        testString.str("[0.2 5.6]");
        testString >> testPoint;

        REQUIRE(testPoint.x == 0.2);
        REQUIRE(testPoint.y == 5.6);
    }

    // Testing Vector2D
    TEST_CASE("Vector2D normalize() works"){ // Zachary, Alves
        Vector2D vec1;
        vec1.x = 7;
        vec1.y = 7;

        Vector2D normVec = normalize(vec1);

        REQUIRE_THAT(normVec.x, WithinAbs(1/sqrt(2), epsilon));
        REQUIRE_THAT(normVec.y, WithinAbs(1/sqrt(2), epsilon));
    }
    TEST_CASE("Vector2D - Point2D works"){ // Zachary, Alves
        Point2D testP1;
        testP1.x = 0.0;
        testP1.y = 0.0;
        Point2D testP2;
        testP2.x = 1.0;
        testP2.y = 1.0;

        Vector2D testVec = testP2 - testP1;

        REQUIRE(testVec.x == 1.0);
        REQUIRE(testVec.y == 1.0);
    }
    TEST_CASE("Vector2D + Point2D works"){ // Zachary, Alves
        Point2D testP;
        testP.x = 0.0;
        testP.y = 0.0;
        Vector2D testV;
        testV.x = 1.0;
        testV.y = 1.0;

        Point2D testPoint = testP + testV;

        REQUIRE(testPoint.x == 1.0);
        REQUIRE(testPoint.y == 1.0);
    }
    TEST_CASE("Vector2D << works"){ // Zachary, Alves
        Vector2D testVec;
        testVec.x = 1.2;
        testVec.y = 3.4;

        std::ostringstream strStream;
        strStream << testVec;

        REQUIRE(strcmp(strStream.str().c_str(), "[1.2 3.4]") == 0);
        REQUIRE(strcmp(strStream.str().c_str(), "[1.2 4.0]") != 0);
    }
    TEST_CASE("Vector2D >> works"){ // Zachary, Alves
        Vector2D testVec;
        testVec.x = 1.2;
        testVec.y = 3.4;

        stringstream testString;
        testString.str("[0.2 5.6]");
        testString >> testVec;

        REQUIRE(testVec.x == 0.2);
        REQUIRE(testVec.y == 5.6);
    }

    // Test cases from HW2

    // Operators
    TEST_CASE("Vector2D += Vector2D works"){ // Zachary, Alves
        Vector2D v1;
        v1.x = 1.0;
        v1.y = 2.0;
        Vector2D v2;
        v2.x = 3.0;
        v2.y = 4.0;

        v1 += v2;

        REQUIRE(v1.x == 4.0);
        REQUIRE(v1.y == 6.0);
    }
    TEST_CASE("Vector2D + Vector2D works"){ // Zachary, Alves
        Vector2D v1;
        v1.x = 1.0;
        v1.y = 2.0;
        Vector2D v2;
        v2.x = 3.0;
        v2.y = 4.0;

        Vector2D v3 = v1 + v2;

        REQUIRE(v3.x == 4.0);
        REQUIRE(v3.y == 6.0);
    }
    TEST_CASE("Vector2D -= Vector2D works"){ // Zachary, Alves
        Vector2D v1;
        v1.x = 1.0;
        v1.y = 2.0;
        Vector2D v2;
        v2.x = 3.0;
        v2.y = 4.0;

        v1 += v2;

        REQUIRE(v1.x == 4.0);
        REQUIRE(v1.y == 6.0);
    }
    TEST_CASE("Vector2D - Vector2D works"){ // Zachary, Alves
        Vector2D v1;
        v1.x = 1.0;
        v1.y = 2.0;
        Vector2D v2;
        v2.x = 3.0;
        v2.y = 4.0;

        Vector2D v3 = v2 - v1;

        REQUIRE(v3.x == 2.0);
        REQUIRE(v3.y == 2.0);
    }
    TEST_CASE("Vector2D *= double works"){ // Zachary, Alves
        Vector2D v1;
        v1.x = 1.0;
        v1.y = 2.0;

        v1 *= 3;

        REQUIRE(v1.x == 3.0);
        REQUIRE(v1.y == 6.0);
    }
    TEST_CASE("Vector2D * double works"){ // Zachary, Alves
        Vector2D v1;
        v1.x = 1.0;
        v1.y = 2.0;

        Vector2D v3 = v1 * 0;

        REQUIRE_THAT(v3.x, WithinAbs(0.0, epsilon));
        REQUIRE_THAT(v3.y, WithinAbs(0.0, epsilon));
        Vector2D v2;
        v2.x = 1.0;
        v2.y = 2.0;

        Vector2D v4 = 0 * v2;

        REQUIRE_THAT(v4.x, WithinAbs(0.0, epsilon));
        REQUIRE_THAT(v4.y, WithinAbs(0.0, epsilon));
    }

    // Functions
    TEST_CASE("dot(Vector2D, Vector2D) works"){ // Zachary, Alves
        Vector2D v1;
        v1.x = 1.0;
        v1.y = 2.0;
        Vector2D v2;
        v2.x = 3.0;
        v2.y = 4.0;

        REQUIRE_THAT(dot(v1, v2), WithinAbs(11.0, epsilon));        
    }
    TEST_CASE("magnitude(Vector2D) works"){ // Zachary, Alves
        Vector2D v1;
        v1.x = 1.0;
        v1.y = 0.0;

        REQUIRE_THAT(magnitude(v1), WithinAbs(1.0, epsilon));

        Vector2D v2;
        v2.x = 0.0;
        v2.y = 0.0;

        REQUIRE_THAT(magnitude(v2), WithinAbs(0.0, epsilon));
    }
    TEST_CASE("angle(Vector2D, Vector2D) works"){ // Zachary, Alves
        Vector2D v1;
        v1.x = 0.0;
        v1.y = 0.0;
        Vector2D v2;
        v2.x = 1.0;
        v2.y = 1.0;

        REQUIRE_THAT(rad2deg(angle(v1, v2)), WithinAbs(45.0, epsilon));
        REQUIRE_THAT(rad2deg(angle(v2, v1)), WithinAbs(-45.0, epsilon));

        Vector2D v3;
        v3.x = 1.0;
        v3.y = -1.0;
        Vector2D v4;
        v4.x = 1.0;
        v4.y = 1.0;

        REQUIRE_THAT(rad2deg(angle(v3, v4)), WithinAbs(90.0, epsilon));

        Vector2D v5;
        v5.x = -1.0;
        v5.y = 1.0;
        Vector2D v6;
        v6.x = -1.0;
        v6.y = -1.0;

        REQUIRE_THAT(rad2deg(angle(v5, v6)), WithinAbs(90.0, epsilon));
    }
}