cmake_minimum_required(VERSION 2.8.3)
project(magnetic_field_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
 tf
)
#find_package(MRPT REQUIRED)
catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(magnetic_field_localization src/magnetic_field_localization.cpp)
add_executable(magnetic_field_vector src/magnetic_field_vector.cpp)
add_executable(magnetic_field_vector_fast src/magnetic_field_vector_fast.cpp src/asa047.cpp)
add_executable(magnetic_field_vector_lis src/magnetic_field_vector_lis.cpp src/asa047.cpp)
add_executable(magnetic_field_localization_two_wire src/magnetic_field_localization_two_wire.cpp src/asa047.cpp)
add_executable(magnetic_field_localization_two_wire_lis src/magnetic_field_localization_two_wire_lis.cpp src/asa047.cpp)


target_link_libraries(magnetic_field_localization
  ${catkin_LIBRARIES}
)

target_link_libraries(magnetic_field_localization_two_wire
  ${catkin_LIBRARIES}
)
target_link_libraries(magnetic_field_localization_two_wire_lis
  ${catkin_LIBRARIES}
)

target_link_libraries(magnetic_field_vector
  ${catkin_LIBRARIES}
)
target_link_libraries(magnetic_field_vector_fast
  ${catkin_LIBRARIES}
)
target_link_libraries(magnetic_field_vector_lis
  ${catkin_LIBRARIES}
)
