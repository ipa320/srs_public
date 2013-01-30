cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)


rosbuild_init()
rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# move arm utils
set( MOVE_ARM_UTILS_SOURCE_FILES src/move_arm_utils/move_arm_utils.cpp
                                 src/move_arm_utils/move_arm_warehouse_logger_reader.cpp )

#set( MOVE_ARM_UTILS_LIB_NAME move_arm_utils )
include_directories(include/move_arm_utils )

# assisted navigation node (action server)
set( ARM_NAV_SOURCE_FILES src/assisted_arm_navigation/arm_manip_node.cpp
                          src/assisted_arm_navigation/arm_manip_action.cpp
                          src/assisted_arm_navigation/arm_manip_collobj_methods.cpp
                          src/assisted_arm_navigation/arm_manip_services.cpp )

set( ARM_NAV_PROJECT_NAME assisted_arm_navigation_node )
include_directories( include/assisted_arm_navigation )
rosbuild_add_executable( ${ARM_NAV_PROJECT_NAME} ${ARM_NAV_SOURCE_FILES} ${MOVE_ARM_UTILS_SOURCE_FILES})


#rosbuild_add_executable( video_flip_node src/assisted_grasping/video_flip.cpp )

# some checks of launch files
rosbuild_add_roslaunch_check(launch ROBOT:=cob3-3)

