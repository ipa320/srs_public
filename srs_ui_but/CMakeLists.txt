cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


rosbuild_init()

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
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})



# BUT rviz display
set( DISPLAY_SOURCE_FILES src/but_display/but_display.cpp
                          src/but_display/but_pointcloud.cpp
                          src/but_display/point_cloud_base.cpp
                          src/but_display/but_examplepane.cpp
                          src/but_display/init.cpp )


set( BUT_DISPLAY_PROJECT_NAME but_gui )
include_directories(include/but_display)
rosbuild_add_library( ${BUT_DISPLAY_PROJECT_NAME} ${DISPLAY_SOURCE_FILES} )

# Add wx widgets
find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

target_link_libraries(${BUT_DISPLAY_PROJECT_NAME} ${wxWidgets_LIBRARIES} ${OGRE_LIBRARIES} )

