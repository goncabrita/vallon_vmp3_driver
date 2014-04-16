cmake_minimum_required(VERSION 2.8.3)
project(vallon_vmp3_driver)

find_package(catkin REQUIRED COMPONENTS roscpp tf metal_detector_msgs cereal_port)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${cereal_port_INCLUDE_DIRS})

add_executable(vallon_vmp3_node src/vallon_driver.cpp)
target_link_libraries(vallon_vmp3_node ${catkin_LIBRARIES} cereal_port)

add_executable(set_coils_zero src/set_coils_zero.cpp)
target_link_libraries(set_coils_zero ${catkin_LIBRARIES})
