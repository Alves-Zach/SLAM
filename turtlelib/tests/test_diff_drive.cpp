#include <catch2/catch_all.hpp>
#include <iostream>
#include <sstream>
#include "turtlelib/diff_drive.hpp"
using namespace std;
using namespace Catch::Matchers;

namespace turtlelib{
    double epsilon = 0.001;

    // Testing DiffDrive
    TEST_CASE("DiffDrive forwardKinematics works"){
        // Creating bot with wheel circumference of 2m
        DiffDrive bot1(1/PI, 0.1);

        // Making the bot move forward 2m
        bot1.forwardKinematics(2 * PI, 2 * PI);

        REQUIRE_THAT(bot1.position().translation().x, WithinAbs(2.0, epsilon));
        REQUIRE_THAT(bot1.position().translation().y, WithinAbs(0.0, epsilon));
        REQUIRE_THAT(bot1.position().rotation(), WithinAbs(0.0, epsilon));
        
        // Creating bot with wheel circumference of 2m
        DiffDrive bot2(1/PI, 0.1);

        // Making the bot move backwards 2m
        bot2.forwardKinematics(-2 * PI, -2 * PI);

        REQUIRE_THAT(bot2.position().translation().x, WithinAbs(-2.0, epsilon));
        REQUIRE_THAT(bot2.position().translation().y, WithinAbs(0.0, epsilon));
        REQUIRE_THAT(bot2.position().rotation(), WithinAbs(0.0, epsilon));

        // Creating bot with wheel circumference of 2m
        DiffDrive bot3(1/PI, 0.1);

        // Making the bot spin 180 degrees
        bot3.forwardKinematics(\
            (1/(2 * bot3.getWheelRadius())) * (-(bot3.getTrack() * PI)),\
            -(1/(2 * bot3.getWheelRadius())) * (-(bot3.getTrack() * PI)));

        REQUIRE_THAT(bot3.position().translation().x, WithinAbs(0.0, epsilon));
        REQUIRE_THAT(bot3.position().translation().y, WithinAbs(0.0, epsilon));
        REQUIRE_THAT(rad2deg(bot3.position().rotation()), WithinAbs(180.0, epsilon));

        // Creating bot with wheel circumference of 2m
        DiffDrive bot4(1/PI, 1/PI);

        // Making the bot follow half of a circle with the center at the left wheel
        bot4.forwardKinematics(0.0, -PI);

        REQUIRE_THAT(bot4.position().translation().x, WithinAbs(0.0, epsilon));
        REQUIRE_THAT(bot4.position().translation().y, WithinAbs(1/PI, epsilon));
        REQUIRE_THAT(bot4.position().rotation(), WithinAbs(-PI, epsilon));
    }
    TEST_CASE("DiffDrive inverseKinematics works"){
        // Creating bot with wheel circumference of 2m
        DiffDrive bot1(1/PI, 0.1);

        // Making the bot move forward 2m
        vector<double> vec1 = bot1.inverseKinematics(Twist2D{0.0, 2.0, 0.0});

        REQUIRE_THAT(vec1.at(0), WithinAbs(2.0 * PI, epsilon));
        REQUIRE_THAT(vec1.at(1), WithinAbs(2.0 * PI, epsilon));

        // Creating bot with wheel circumference of 2m
        DiffDrive bot2(1/PI, 0.1);

        // Giving it an impossible twist and catching the logic error
        try{
            vector<double> vec2 = bot2.inverseKinematics(Twist2D{0.0, 0.0, 1.0});
        }catch (std::logic_error & L){
            REQUIRE(strcmp("The robot can't move in the y direction alone", L.what()) == 0);
        }

        // Creating bot with wheel circumference of 2m
        DiffDrive bot3(1/PI, PI/10);

        // Making the bot spin 180 degrees
        vector<double> vec3 = bot3.inverseKinematics(Twist2D{PI, 0.0, 0.0});

        REQUIRE_THAT(vec3.at(0),\
            WithinAbs((1/(2 * bot3.getWheelRadius())) * (-(bot3.getTrack() * PI)), epsilon));
        REQUIRE_THAT(vec3.at(1),\
            WithinAbs((1/(2 * bot3.getWheelRadius())) * ((bot3.getTrack() * PI)), epsilon));
    }
}