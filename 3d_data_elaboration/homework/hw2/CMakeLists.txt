cmake_minimum_required(VERSION 2.8)
project(lab1)
find_package(PCL REQUIRED)

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable (es1 src/es1.cpp)
target_link_libraries(es1 ${PCL_LIBRARIES})

add_executable (es2 src/es2.cpp)
target_link_libraries(es2 ${PCL_LIBRARIES})

add_executable (es3 src/es3.cpp)
target_link_libraries(es3 ${PCL_LIBRARIES})

add_executable (es4 src/es4.cpp)
target_link_libraries(es4 ${PCL_LIBRARIES})

add_executable (es5_registration src/es5_registration.cpp)
target_link_libraries(es5_registration ${PCL_LIBRARIES})

add_executable (es5_robot_extraction src/es5_robot_extraction.cpp)
target_link_libraries(es5_robot_extraction ${PCL_LIBRARIES})

add_executable (es6 src/es6.cpp)
target_link_libraries(es6 ${PCL_LIBRARIES})



