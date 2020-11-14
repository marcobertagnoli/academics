cmake_minimum_required(VERSION 2.8)
project(lab2)
find_package(PCL REQUIRED)

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

add_executable (demo_compute_normals src/demo_compute_normals.cpp)
target_link_libraries(demo_compute_normals ${PCL_LIBRARIES})

add_executable (demo_compute_FPFH src/demo_compute_FPFH.cpp)
target_link_libraries(demo_compute_FPFH ${PCL_LIBRARIES} pthread)

add_executable (openni_simple_viewer src/openni_simple_viewer.cpp)
target_link_libraries (openni_simple_viewer ${PCL_LIBRARIES} pthread)

#add_executable (es1 src/es1.cpp)
#target_link_libraries (es1 ${PCL_LIBRARIES} pthread)

