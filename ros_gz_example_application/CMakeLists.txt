cmake_minimum_required(VERSION 3.5)
project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  PROGRAMS
    scripts/ball_spawner.py
    scripts/go_to_goal.py
    scripts/path_publisher.py
    scripts/initForest.py
    scripts/handleFire.py
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
