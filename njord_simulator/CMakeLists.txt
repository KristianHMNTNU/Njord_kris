cmake_minimum_required(VERSION 3.10)
project(njord_simulator)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch config urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    ${PROJECT_NAME}/munin.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
