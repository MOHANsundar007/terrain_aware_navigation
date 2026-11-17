cmake_minimum_required(VERSION 3.8)
project(terrain_nav_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_program(XACRO_EXE xacro)
  find_program(CHECK_URDF_EXE check_urdf)
  if(XACRO_EXE AND CHECK_URDF_EXE)
    add_test(
      NAME xacro_check_urdf
      COMMAND sh -c "${XACRO_EXE} ${CMAKE_CURRENT_SOURCE_DIR}/urdf/rover.urdf.xacro > ${CMAKE_CURRENT_BINARY_DIR}/rover.urdf && ${CHECK_URDF_EXE} ${CMAKE_CURRENT_BINARY_DIR}/rover.urdf"
    )
  endif()
endif()

ament_package()
