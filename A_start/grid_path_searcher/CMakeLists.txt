cmake_minimum_required(VERSION 3.0.2)
project(grid_path_searcher)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# release
SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
        INCLUDE_DIRS include
)

include_directories(
        include
        SYSTEM
        third_party
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

add_executable( demo_node
        src/demo_node.cpp
        src/Astar_searcher.cpp
        )

target_link_libraries(demo_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable ( random_complex
        src/random_complex_generator.cpp )

target_link_libraries( random_complex
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES} )