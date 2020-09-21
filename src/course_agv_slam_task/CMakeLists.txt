cmake_minimum_required(VERSION 2.8.3)
project(course_agv_slam_task)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  tf
  tf_conversions
)
find_package(Eigen3 REQUIRED)

add_definitions(-std=c++11 -O2 -g -DNDEBUG)

catkin_package (
#	CATKIN_DEPENDS roscpp
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(icp src/icp.cpp)
target_link_libraries(icp 
${catkin_LIBRARIES}
)

add_executable(extraction src/extraction.cpp)
target_link_libraries(extraction
${catkin_LIBRARIES}
)

add_executable(icp_lm src/icp_lm.cpp)
target_link_libraries(icp_lm
${catkin_LIBRARIES}
)

add_executable(ekf src/ekf.cpp)
target_link_libraries(ekf
${catkin_LIBRARIES}
)

add_executable(mapping src/mapping.cpp)
target_link_libraries(mapping
${catkin_LIBRARIES}
)

add_executable(bayes_mapping src/bayes_mapping.cpp)
target_link_libraries(bayes_mapping
${catkin_LIBRARIES}
)

add_executable(particle_filter src/particle_filter.cpp)
target_link_libraries(particle_filter
${catkin_LIBRARIES}
)
