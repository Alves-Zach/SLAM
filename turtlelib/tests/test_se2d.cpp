#include <catch2/catch_all.hpp>
#include <iostream>
#include <sstream>
#include "turtlelib/se2d.hpp"
using namespace std;
using namespace Catch::Matchers;

namespace turtlelib{
    double epsilon = 0.001;

    // Testing Twist2D
    TEST_CASE("Twist2D << works"){ // Zachary, Alves
        Twist2D testTwist;
        testTwist.omega = 90;
        testTwist.x = 1.2;
        testTwist.y = 3.4;

        std::ostringstream strStream1;
        strStream1 << testTwist;

        REQUIRE(strcmp(strStream1.str().c_str(), "[90 1.2 3.4]") == 0);
        REQUIRE(strcmp(strStream1.str().c_str(), "[20 3.0 4.0]") != 0);

        testTwist.y = -3.4;
        std::ostringstream strStream2;
        strStream2 << testTwist;
        REQUIRE(strcmp(strStream2.str().c_str(), "[90 1.2 -3.4]") == 0);
    }
    TEST_CASE("Twist2D >> works"){ // Zachary, Alves
        Twist2D testTwist;
        testTwist.omega = 1.2;
        testTwist.x = 1.2;
        testTwist.y = 3.4;

        stringstream testString;
        testString.str("[45 0.2 5.6]");
        testString >> testTwist;

        REQUIRE(testTwist.omega == 45);
        REQUIRE(testTwist.x == 0.2);
        REQUIRE(testTwist.y == 5.6);

        testString.str("-0.5 -0.2 -5.6");
        testString >> testTwist;
        REQUIRE(testTwist.omega == -0.5);
        REQUIRE(testTwist.x == -0.2);
        REQUIRE(testTwist.y == -5.6);
    }

    /// \test Transform2D testing

    // Testing Transform2D Constructors
    TEST_CASE("Transform2D default constructor works"){ // Zachary, Alves
        Transform2D testTransform;

        std::ostringstream strStream1;
        strStream1 << testTransform;

        REQUIRE_THAT(testTransform.rotation(), WithinAbs(0.0, epsilon));
        REQUIRE_THAT(testTransform.translation().x, WithinAbs(0.0, epsilon));
        REQUIRE_THAT(testTransform.translation().y, WithinAbs(0.0, epsilon));
    }
    TEST_CASE("Transform2D Vector2D constructor works"){ // Zachary, Alves
        Vector2D vec1;
        vec1.x = 2;
        vec1.y = 3;

        Transform2D testTransform1(vec1);

        std::ostringstream strStream1;
        strStream1 << testTransform1;

        REQUIRE_THAT(testTransform1.rotation(), WithinAbs(0.0, epsilon));
        REQUIRE_THAT(testTransform1.translation().x, WithinAbs(2.0, epsilon));
        REQUIRE_THAT(testTransform1.translation().y, WithinAbs(3.0, epsilon));

        Vector2D vec2;
        vec2.x = -2.3;
        vec2.y = -0.8;

        Transform2D testTransform2(vec2);

        std::ostringstream strStream2;
        strStream2 << testTransform2;

        REQUIRE_THAT(testTransform2.rotation(), WithinAbs(0.0, epsilon));
        REQUIRE_THAT(testTransform2.translation().x, WithinAbs(-2.3, epsilon));
        REQUIRE_THAT(testTransform2.translation().y, WithinAbs(-0.8, epsilon));
    }
    TEST_CASE("Transform2D double constructor works"){ // Zachary, Alves
        Transform2D testTrans1(0.0);

        REQUIRE(testTrans1.rotation() == 0.0);

        Transform2D testTrans2(PI);

        REQUIRE(testTrans2.rotation() == PI);
    }
    TEST_CASE("Transform2D Vector2D and double constructor works"){ // Zachary, Alves
        Vector2D vec1;
        vec1.x = 2;
        vec1.y = 3;
        Transform2D testTrans1(vec1, 0.0);

        std::ostringstream strStream1;
        strStream1 << testTrans1;

        REQUIRE_THAT(testTrans1.rotation(), WithinAbs(0.0, epsilon));
        REQUIRE_THAT(testTrans1.translation().x, WithinAbs(2.0, epsilon));
        REQUIRE_THAT(testTrans1.translation().y, WithinAbs(3.0, epsilon));

        Transform2D testTrans2(vec1, PI);

        REQUIRE_THAT(testTrans2.rotation(), WithinAbs(PI, epsilon));
        REQUIRE_THAT(testTrans2.translation().x, WithinAbs(2.0, epsilon));
        REQUIRE_THAT(testTrans2.translation().y, WithinAbs(3.0, epsilon));
    }
    
    // Testing Transform2D() operations on other objects
    TEST_CASE("Transform2D(Point2D) works"){ // Zachary, Alves
        Transform2D rot90(PI/2);
        Point2D test;
        test.x = 4;
        test.y = 6;

        Point2D newPoint = rot90(test);

        REQUIRE_THAT(newPoint.x, WithinAbs(-6, epsilon));
        REQUIRE_THAT(newPoint.y, WithinAbs(4, epsilon));
    }
    TEST_CASE("Transform2D(Vector2D) works"){ // Zachary, Alves
        Vector2D vec1;
        vec1.x = 3;
        vec1.y = 5;

        Transform2D testTransform1(PI/2);
        Vector2D vec2 = testTransform1(vec1);

        REQUIRE_THAT(vec2.x, WithinAbs(-5, epsilon));
        REQUIRE_THAT(vec2.y, WithinAbs(3, epsilon));
    }
    TEST_CASE("Transform2D(Twist2D) works"){ // Zachary, Alves
        Twist2D twist1;
        twist1.x = 3;
        twist1.y = 5;
        twist1.omega = PI;

        Transform2D testTransform1(PI/2);
        Twist2D twist2 = testTransform1(twist1);

        REQUIRE_THAT(twist2.omega, WithinAbs(twist1.omega, epsilon));
        REQUIRE_THAT(twist2.x, WithinAbs(-5, epsilon));
        REQUIRE_THAT(twist2.y, WithinAbs(3, epsilon));        
    }

    // Testing Transform2D Functions
    TEST_CASE("Transform2D inv works"){ // Zachary, Alves
        Vector2D vec1;
        vec1.x = 3;
        vec1.y = 3;

        Transform2D testTransform1(vec1, PI/2);

        Transform2D invt1 = testTransform1.inv();

        REQUIRE_THAT(invt1.rotation(), WithinAbs(-PI/2, epsilon));
        REQUIRE_THAT(invt1.translation().x, WithinAbs(-3, epsilon));
        REQUIRE_THAT(invt1.translation().y, WithinAbs(3, epsilon));
    }
    TEST_CASE("Transform2D translation works"){ // Zachary, Alves
        Vector2D vec1;
        vec1.x = 5;
        vec1.y = 6;

        Transform2D testTransform1(vec1, 10);

        REQUIRE_THAT(testTransform1.translation().x, WithinAbs(5, epsilon));
        REQUIRE_THAT(testTransform1.translation().y, WithinAbs(6, epsilon));

        Vector2D vec2;
        vec2.x = 2;
        vec2.y = 3;

        Transform2D testTransform2(vec2);

        REQUIRE_THAT(testTransform2.translation().x, WithinAbs(2, epsilon));
        REQUIRE_THAT(testTransform2.translation().y, WithinAbs(3, epsilon));

        Transform2D testTransform3;

        REQUIRE_THAT(testTransform3.translation().x, WithinAbs(0, epsilon));
        REQUIRE_THAT(testTransform3.translation().y, WithinAbs(0, epsilon));
    }
    TEST_CASE("Transform2D rotation works"){ // Zachary, Alves
        Vector2D vec;
        vec.x = 2;
        vec.y = 3;

        Transform2D testTransform(vec);

        REQUIRE_THAT(testTransform.rotation(), WithinAbs(0, epsilon));
    }

    // Testing Transform2D Operators
    TEST_CASE("Transform2D << works"){ // Zachary, Alves
        Transform2D testTransform;

        std::ostringstream strStream1;
        strStream1 << testTransform;

        REQUIRE(strcmp(strStream1.str().c_str(),
                        "deg: 0 x: 0 y: 0") == 0);
    }
    TEST_CASE("Transform2D >> works"){ // Zachary, Alves
        Transform2D testT1;

        stringstream testString;
        testString.str("[0 0.2 5.6]");
        testString >> testT1;

        std::ostringstream strStream1;
        strStream1 << testT1;

        REQUIRE_THAT(testT1.rotation(), WithinAbs(0, epsilon));
        REQUIRE_THAT(testT1.translation().x, WithinAbs(0.2, epsilon));
        REQUIRE_THAT(testT1.translation().y, WithinAbs(5.6, epsilon));

        Transform2D testT2;
        testString.str("-90 -0.2 -5.6");
        testString >> testT2;

        std::ostringstream strStream2;
        strStream2 << testT2;

        REQUIRE_THAT(testT2.rotation(), WithinAbs(-PI/2, epsilon));
        REQUIRE_THAT(testT2.translation().x, WithinAbs(-0.2, epsilon));
        REQUIRE_THAT(testT2.translation().y, WithinAbs(-5.6, epsilon));
    }
    TEST_CASE("Transform2D *= works"){ // Zachary, Alves
        Vector2D vec;
        vec.x = 1;
        vec.y = 2;

        Transform2D I; // Identity transformation matrix
        Transform2D test(vec, 2);

        std::ostringstream strStream1;
        strStream1 << test;

        test *= I;

        std::ostringstream strStream2;
        strStream2 << test;

        REQUIRE(strcmp(strStream1.str().c_str(),
                        strStream2.str().c_str()) == 0);
    }
    TEST_CASE("Transform2D * Transform2D works"){ // Zachary, Alves
        Vector2D vec;
        vec.x = 1;
        vec.y = 2;

        Transform2D I; // Identity transformation matrix
        Transform2D test(vec, 2);

        std::ostringstream strStream1;
        strStream1 << test;

        Transform2D newT = test * I;

        std::ostringstream strStream2;
        strStream2 << newT;

        REQUIRE(strcmp(strStream1.str().c_str(),
                        strStream2.str().c_str()) == 0);
    }

    // Added in HW2
    TEST_CASE("Transform2D integrate_twist works"){ // Zachary, Alves
        Twist2D tw1;
        tw1.omega = 0.0;
        tw1.x = 1.0;
        tw1.y = 1.0;

        Transform2D trans1 = integrate_twist(tw1);

        REQUIRE_THAT(trans1.rotation(), WithinAbs(0.0, epsilon));
        REQUIRE_THAT(trans1.translation().x, WithinAbs(1.0, epsilon));
        REQUIRE_THAT(trans1.translation().y, WithinAbs(1.0, epsilon));

        Twist2D tw2;
        tw2.omega = PI;
        tw2.x = 0.0;
        tw2.y = 0.0;

        Transform2D trans2 = integrate_twist(tw2);

        REQUIRE_THAT(trans2.rotation(), WithinAbs(PI, epsilon));
        REQUIRE_THAT(trans2.translation().x, WithinAbs(0.0, epsilon));
        REQUIRE_THAT(trans2.translation().y, WithinAbs(0.0, epsilon));
    }

    // Classmate's test cases
    TEST_CASE("integrate_twist", "Twist2D"){ // Stella, Yu
        // Only Translation
        Twist2D tw = Twist2D{0, 1, 2};
        Transform2D Tbbprime = integrate_twist(tw);
        REQUIRE_THAT(Tbbprime.translation().x, Catch::Matchers::WithinAbs(1.0, 1e-5));
        REQUIRE_THAT(Tbbprime.translation().y, Catch::Matchers::WithinAbs(2.0, 1e-5));
        REQUIRE_THAT(Tbbprime.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-5));
        // Only Rotation
        tw = Twist2D{PI, 0, 0};
        Tbbprime = integrate_twist(tw);
        REQUIRE_THAT(Tbbprime.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(Tbbprime.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(Tbbprime.rotation(), Catch::Matchers::WithinAbs(PI, 1e-5));
        // Translation + Rotation
        tw = Twist2D{-1.24, -2.15,-2.92};
        Tbbprime = integrate_twist(tw);
        REQUIRE_THAT(Tbbprime.translation().x, Catch::Matchers::WithinAbs(-3.229863264722, 1e-5));
        REQUIRE_THAT(Tbbprime.translation().y, Catch::Matchers::WithinAbs(-1.05645265317421, 1e-5));
        REQUIRE_THAT(Tbbprime.rotation(), Catch::Matchers::WithinAbs(-1.24, 1e-5));
    }
}