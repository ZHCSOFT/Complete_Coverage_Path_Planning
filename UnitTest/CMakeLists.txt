cmake_minimum_required(VERSION 3.5)
project(test_cccp)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 11)

add_executable(${PROJECT_NAME}_boustrophedon
    test_boustrophedon.cpp
)

target_link_libraries(${PROJECT_NAME}_boustrophedon
    cccp_algorithm
)

target_include_directories(${PROJECT_NAME}_boustrophedon PRIVATE
    ${CMAKE_SOURCE_DIR}/algorithms/include
    ${CMAKE_SOURCE_DIR}/utils/include
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/thirdparties/yaml-cpp/include
)
