cmake_minimum_required(VERSION 2.8)
project(lab4)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#add_executable(peopleDetector src/main_ground_based_people_detection.cpp)
#target_link_libraries(peopleDetector ${PCL_LIBRARIES} vtkHybrid vtkRendering)

add_executable(show_clouds_nao src/main_show_clouds_nao.cpp)
target_link_libraries(show_clouds_nao ${PCL_LIBRARIES} vtkHybrid vtkRendering)

#add_executable (cloud_registration src/cloud_registration.cpp)
#target_link_libraries(cloud_registration ${PCL_LIBRARIES})

#add_executable (plane_segmentation src/plane_segmentation.cpp)
#target_link_libraries(plane_segmentation ${PCL_LIBRARIES})

add_executable (people_detector src/people_detector.cpp)
target_link_libraries(people_detector ${PCL_LIBRARIES})
