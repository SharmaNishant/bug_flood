cmake_minimum_required(VERSION 3.3)
project(VisibilityGraphsNewCommit)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")


set(SOURCE_FILES main.cpp point.cpp line.cpp graphutility.cpp obstacles.cpp pointHandler.cpp dijkstra.h dijkstra.cpp VisibilityGraph.cpp VisibilityGraphController.cpp geometry.h boostHelper.h CImg.h  utility.h drawing.h point.h line.h graphutility.h obstacles.h obstacleController.h pointHandler.h VisibilityGraph.h VisibilityGraphController.h dj.h.h dj.cpp.cpp obstacleGeneration.h)
add_executable(VisibilityGraphsNewCommit ${SOURCE_FILES})
target_link_libraries(VisibilityGraphsNewCommit -lX11 -pthread -lboost_regex )
