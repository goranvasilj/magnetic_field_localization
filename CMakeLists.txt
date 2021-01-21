cmake_minimum_required(VERSION 2.8.3)
project(magnetic_field_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
 tf
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(magnetic_field_localization src/magnetic_field_localization.cpp)
add_executable(magnetic_field_vector src/magnetic_field_vector.cpp)


target_link_libraries(magnetic_field_localization
  ${catkin_LIBRARIES}
)
target_link_libraries(magnetic_field_vector
  ${catkin_LIBRARIES}
)
