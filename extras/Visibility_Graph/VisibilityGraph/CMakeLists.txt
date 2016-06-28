cmake_minimum_required(VERSION 3.3)
project(VisibilityGraph)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES src/main.cpp src/point.cpp src/line.cpp src/graphutility.cpp src/obstacles.cpp src/pointHandler.cpp include/dijkstra.h src/dijkstra.cpp src/VisibilityGraph.cpp src/VisibilityGraphController.cpp include/geometry.h include/boostHelper.h include/CImg.h include/utility.h include/drawing.h include/point.h include/line.h include/graphutility.h include/obstacles.h include/obstacleController.h include/pointHandler.h include/VisibilityGraph.h include/VisibilityGraphController.h include/dj.h.h src/dj.cpp.cpp include/obstacleGeneration.h)
add_executable(VisibilityGraph ${SOURCE_FILES})
target_link_libraries(VisibilityGraph -lX11 -pthread )
