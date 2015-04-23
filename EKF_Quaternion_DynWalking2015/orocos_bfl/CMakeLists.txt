# Copyright (C) 2015 CoDyCo
# Author: Jorhabib Eljaik
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME quaternionEKF)
project(${PROJECTNAME})

find_package(YARP   REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
message("** Proj source dir " ${CMAKE_CURRENT_SOURCE_DIR})
# include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindMATRIX.cmake)
# MATRIX also looks for Boost
find_package(MATRIX REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)

# Find orocos-bfl package configuration file
find_package(PkgConfig)
pkg_check_modules(OROCOS_BFL REQUIRED orocos-bfl)

file(GLOB source_dir ./src/*.cpp)
file(GLOB header_dir ./include/*.h)

source_group("Source Files" FILES ${source_dir})
source_group("Header Files" FILES ${header_dir})

include_directories(SYSTEM ${YARP_INCLUDE_DIRS} 
                           ${MATRIX_INCLUDE}
                           ${OROCOS_BFL_INCLUDEDIR}
                           ${Boost_INCLUDE_DIRS}
                           ${PROJECT_SOURCE_DIR}/include)

message("** Matrix include dirs are: " ${MATRIX_INCLUDE})
message("** Orocos-bfl include dirs are: " ${OROCOS_BFL_INCLUDEDIR})
message("** Boost include dirs are: " ${Boost_INCLUDE_DIRS})
message("** Boost libraries are: ${Boost_LIBRARIES}")

add_executable(${PROJECTNAME} ${source_dir} ${header_dir})
target_link_libraries(${PROJECTNAME} 
                      orocos-bfl
	              ${YARP_LIBRARIES}
	              ${MATRIX_LIBS}
	              ${Boost_LIBRARIES})
