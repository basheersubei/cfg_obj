cmake_minimum_required(VERSION 2.8.3)
project(cfg_obj)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(PCL 1.3 REQUIRED)


include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ransac src/ransac.cpp)
target_link_libraries ( ransac ${PCL_LIBRARIES} ${catkin_LIBRARIES} )

##########

#rosbuild_add_executable(ransac src/ransac.cpp)
#rosbuild_add_executable(scan src/scan.cpp)
#rosbuild_add_executable(test src/test.cpp)
#target_link_libraries(example ${PROJECT_NAME})
