cmake_minimum_required(VERSION 3.5)
project(ur_server)

find_package(catkin REQUIRED COMPONENTS roscpp roslaunch actionlib actionlib_msgs)
find_package(Boost REQUIRED COMPONENTS system)
find_package(ur_rtde REQUIRED)


add_action_files(
  DIRECTORY action
  FILES ur.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})



add_executable(ur_server src/ur_server.cpp)
add_executable(ur_client src/ur_client.cpp)

target_link_libraries(ur_server PRIVATE ${catkin_LIBRARIES} ur_rtde::rtde)
target_link_libraries(ur_client ${catkin_LIBRARIES})

#add_dependencies(fibonacci_client ${actionlib_tutorials_EXPORTED_TARGETS})
#target_link_libraries(ur_server PRIVATE ur_rtde::rtde)

#add_dependencies(ur_server ${actionlib_tutorials_EXPORTED_TARGETS})





#add_executable(ur_path src/trajectory.cpp)
#add_executable(ur_point src/move_to_point.cpp)
#add_executable(readfile src/readfile.cpp)


