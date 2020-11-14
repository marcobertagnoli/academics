cmake_minimum_required(VERSION 2.8)
project(lab1)
find_package(PCL REQUIRED)

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable (demo_simple_operations src/demo_simple_operations.cpp)
target_link_libraries(demo_simple_operations ${PCL_LIBRARIES})

add_executable (demo_simple_operations_b src/demo_simple_operations_b.cpp)
target_link_libraries(demo_simple_operations_b ${PCL_LIBRARIES})

add_executable (demo_voxel_grid_filtering src/demo_voxel_grid_filtering.cpp)
target_link_libraries(demo_voxel_grid_filtering ${PCL_LIBRARIES})

add_executable (es1 src/es1.cpp)
target_link_libraries(es1 ${PCL_LIBRARIES})



