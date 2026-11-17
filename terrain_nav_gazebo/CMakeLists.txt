cmake_minimum_required(VERSION 3.8)
project(terrain_nav_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds launch config models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/generate_terrain.py DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
