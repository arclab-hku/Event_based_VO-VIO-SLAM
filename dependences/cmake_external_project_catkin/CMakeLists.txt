cmake_minimum_required(VERSION 2.8.3)
project(cmake_external_project_catkin)

find_package(catkin REQUIRED)

# catkin_package(
#   CATKIN_DEPENDS catkin
#   CFG_EXTRAS cmake_external_project_catkin-extras.cmake
# )

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_install()
cs_export(CFG_EXTRAS cmake_external_project_catkin-extras.cmake)
