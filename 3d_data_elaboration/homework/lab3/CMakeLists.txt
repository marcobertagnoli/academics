cmake_minimum_required(VERSION 2.8)
project(lab3)
find_package(PCL 1.7 REQUIRED)

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

add_executable (pairwise_registration src/pairwise_registration.cpp)
target_link_libraries (pairwise_registration ${PCL_LIBRARIES} vtkHybrid vtkRendering)

add_executable (plane_segmentation src/plane_segmentation.cpp)
target_link_libraries (plane_segmentation ${PCL_LIBRARIES} vtkHybrid vtkRendering)

add_executable (dataset_visualizer src/dataset_visualizer.cpp)
target_link_libraries(dataset_visualizer ${PCL_LIBRARIES})


