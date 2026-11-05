cmake_minimum_required(VERSION 3.8)
project(hexapod_gazebo)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch worlds config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
ament_package()
