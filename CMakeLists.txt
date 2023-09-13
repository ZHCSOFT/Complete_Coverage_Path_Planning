cmake_minimum_required(VERSION 3.5)
project(cccp_algorithm)

set(CMAKE_BUILD_TYPE "Debug")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system chrono thread)
find_package(PkgConfig REQUIRED)

add_library(${PROJECT_NAME} SHARED
    algorithms/src/boustrophedon_explorator.cpp
    algorithms/src/energy_functional_explorator.cpp

    utils/src/ipa_room_exploration/meanshift2d.cpp
    utils/src/ipa_room_exploration/room_rotator.cpp
    utils/src/ipa_room_exploration/fov_to_robot_mapper.cpp

    utils/src/ipa_building_navigation/A_star_pathplanner.cpp
    utils/src/ipa_building_navigation/node.cpp
    utils/src/ipa_building_navigation/genetic_TSP.cpp
    utils/src/ipa_building_navigation/nearest_neighbor_TSP.cpp

    utils/src/ros/map_accessibility_analysis.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    algorithms/include
    utils/include
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)

add_subdirectory(thirdparties)
add_subdirectory(UnitTest)
