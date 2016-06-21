cmake_minimum_required(VERSION 3.3)
project(Generation_Bug_Flood)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/obstacleGeneration")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
FIND_PACKAGE( OpenCV )
set(SOURCE_FILES main.cpp obstacleGeneration/obstacle_generation.cpp)
add_executable(Generation_Bug_Flood ${SOURCE_FILES})

target_link_libraries(Generation_Bug_Flood ${OpenCV_LIBS})
