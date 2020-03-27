cmake_minimum_required(VERSION 2.8.3)
project(following_the_path)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
)

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(following_the_path src/following_the_path.cpp)
target_link_libraries(following_the_path ${catkin_LIBRARIES})
