cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install the launch and urdf folders
install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
