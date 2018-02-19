cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(phidgets_encoder)

find_package(Boost REQUIRED)

find_package(
	catkin REQUIRED
	COMPONENTS
		phidgets_api
		roscpp
		sensor_msgs
)

catkin_package(DEPENDS Boost)

include_directories(include/ ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(phidgets_encoder_node src/phidgets_encoder_node.cpp
               src/phidgets_wheels_publisher.cpp src/exceptions.cpp)
target_link_libraries(phidgets_encoder_node ${catkin_LIBRARIES})
add_dependencies(phidgets_encoder_node ${catkin_EXPORTED_TARGETS})

install(TARGETS phidgets_encoder_node
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
