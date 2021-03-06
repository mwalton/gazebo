cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(model_push SHARED model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(avoid_walls SHARED avoid_walls.cc)
target_link_libraries(avoid_walls ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(move_foreward SHARED move_foreward.cc)
target_link_libraries(move_foreward ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
