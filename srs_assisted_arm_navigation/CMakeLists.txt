cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()
rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

# BUT rviz display
set( DISPLAY_SOURCE_FILES src/rviz_plugins/arm_navigation_controls.cpp
                          src/rviz_plugins/arm_navigation_display.cpp
                          src/rviz_plugins/init.cpp
                          src/rviz_plugins/grasping_controls.cpp
                          src/rviz_plugins/grasping_display.cpp
                          src/rviz_plugins/bb_estimation_controls.cpp
                          src/rviz_plugins/bb_estimation_display.cpp )

set( BUT_DISPLAY_PROJECT_NAME but_assisted_plugins )
include_directories(include/rviz_plugins)
#include_directories(include/assisted_grasping_plugin)
rosbuild_add_library( ${BUT_DISPLAY_PROJECT_NAME} ${DISPLAY_SOURCE_FILES} )


# Add wx widgets
find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

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

# assisted grasping
set( GRASPING_SOURCE_FILES src/assisted_grasping/manual_grasping_node.cpp )

set( GRASPING_PROJECT_NAME assisted_grasping_node )
include_directories( include/assisted_grasping )
rosbuild_add_executable( ${GRASPING_PROJECT_NAME} ${GRASPING_SOURCE_FILES} )

rosbuild_add_executable( video_flip_node src/assisted_grasping/video_flip.cpp )

# some checks of launch files
rosbuild_add_roslaunch_check(launch)

