cmake_minimum_required(VERSION 2.8.3)
project(agitr)
find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(hello hello.cpp)
add_executable(count count.cpp)
add_executable(once once.cpp)
add_executable(throttle throttle.cpp)
add_executable(reverse_cmd_vel reverse_cmd_vel.cpp)
target_link_libraries(throttle ${catkin_LIBRARIES})
target_link_libraries(hello ${catkin_LIBRARIES})
target_link_libraries(count ${catkin_LIBRARIES})
target_link_libraries(once ${catkin_LIBRARIES})
target_link_libraries(reverse_cmd_vel ${catkin_LIBRARIES})
