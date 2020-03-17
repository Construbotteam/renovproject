cmake_minimum_required(VERSION 2.8.3)
project(mobile_base_description)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package(
  CATKIN_DEPENDS urdf xacro
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
