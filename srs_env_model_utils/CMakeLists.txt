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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_boost_directories()

# Robot self filter
rosbuild_add_library( robot_self_filter src/self_mask.cpp )
rosbuild_add_openmp_flags( robot_self_filter )

rosbuild_add_executable( test_filter src/test_filter.cpp )
rosbuild_add_openmp_flags( test_filter )
target_link_libraries( test_filter robot_self_filter )

rosbuild_add_executable( self_filter src/self_filter.cpp )
rosbuild_add_openmp_flags( self_filter )
target_link_libraries( self_filter robot_self_filter )

# Kinect throttling nodelets
rosbuild_add_library( kinect_throttle 
                      src/cloud_throttle.cpp 
                      src/kinect_throttle.cpp 
                      )

