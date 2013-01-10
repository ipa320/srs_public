cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


# Add wx widgets
find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

# Boost
rosbuild_add_boost_directories()


# BUT rviz display
set( BUT_DISPLAY_SOURCE_FILES src/but_rviz_display/cam_publisher_display.cpp
                              src/but_rviz_display/octomap_display.cpp
                              src/but_rviz_display/octomap_control_pane.cpp
                              src/but_rviz_display/point_cloud_display.cpp
                              src/but_rviz_display/point_cloud_base.cpp
                              src/init.cpp 
                              )

set( BUT_DISPLAY_PROJECT_NAME but_env_model_gui )
rosbuild_add_library( ${BUT_DISPLAY_PROJECT_NAME} ${BUT_DISPLAY_SOURCE_FILES} )

target_link_libraries( ${BUT_DISPLAY_PROJECT_NAME} ${wxWidgets_LIBRARIES} ${OGRE_LIBRARIES} )
rosbuild_link_boost( ${BUT_DISPLAY_PROJECT_NAME} thread )

