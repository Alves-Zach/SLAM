cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Doxygen)
find_package(nuturtlebot_msgs REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

option(BUILD_DOCS "Build the documentation" OFF)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Twist.msg"
  "srv/InitialPose.srv"
  "srv/Control.srv"
  "srv/Straight.srv"
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

######## Begin_citation [1] #
add_executable(turtle_control src/turtle_control.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(circle src/circle.cpp)

target_link_libraries(turtle_control turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(odometry turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(circle turtlelib::turtlelib "${cpp_typesupport_target}")

ament_target_dependencies(turtle_control
                          rclcpp
                          geometry_msgs
                          sensor_msgs
                          nuturtlebot_msgs
                          turtlelib)
ament_target_dependencies(odometry
                          rclcpp
                          geometry_msgs
                          sensor_msgs
                          nuturtlebot_msgs
                          tf2
                          tf2_ros
                          nav_msgs
                          turtlelib)
ament_target_dependencies(circle
                          rclcpp
                          geometry_msgs
                          std_srvs
                          turtlelib)
######## End_citation [1] #

install(TARGETS
    turtle_control
    odometry
    circle
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
launch
config
DESTINATION share/${PROJECT_NAME})

ament_package()
