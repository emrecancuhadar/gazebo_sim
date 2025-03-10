cmake_minimum_required(VERSION 3.5)
project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
    DIRECTORY src/
    DESTINATION lib/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
)

ament_package()
