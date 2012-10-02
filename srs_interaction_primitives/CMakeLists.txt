cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# BUT gui primitives
set( INTERACTION_PRIMITIVES_SOURCE_FILES 
     src/interaction_primitives/bounding_box.cpp
     src/interaction_primitives/primitive.cpp
     src/interaction_primitives/billboard.cpp
     src/interaction_primitives/plane.cpp
     src/interaction_primitives/plane_polygon.cpp
     src/interaction_primitives/object.cpp
     src/interaction_primitives/unknown_object.cpp
     src/interaction_primitives/update_publisher.cpp
     src/clickable_positions/clickable_positions.cpp )

set( INTERACTION_PRIMITIVES_LIB_NAME but_interaction_primitives )
rosbuild_add_library( ${INTERACTION_PRIMITIVES_LIB_NAME} ${INTERACTION_PRIMITIVES_SOURCE_FILES} )

rosbuild_add_executable( interaction_primitives_service_server src/nodes/service_server.cpp )
rosbuild_add_executable( clickable_positions_action_server src/nodes/clickable_positions_action_server.cpp )
rosbuild_add_executable( test_primitives src/test/test_primitives.cpp )
rosbuild_add_executable( test_observers src/test/test_observers.cpp )

target_link_libraries( interaction_primitives_service_server ${INTERACTION_PRIMITIVES_LIB_NAME} )
target_link_libraries( clickable_positions_action_server ${INTERACTION_PRIMITIVES_LIB_NAME} )
target_link_libraries( test_primitives ${INTERACTION_PRIMITIVES_LIB_NAME} )
target_link_libraries( test_observers ${INTERACTION_PRIMITIVES_LIB_NAME} )
