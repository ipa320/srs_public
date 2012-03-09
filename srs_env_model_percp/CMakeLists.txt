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
rosbuild_gensrv()

# Plane detection
rosbuild_add_executable(segmenter src/plane_det/segmenter.cpp)
rosbuild_add_executable(segmenter src/plane_det/filtering.cpp)
rosbuild_add_executable(segmenter src/plane_det/normals.cpp)
rosbuild_add_executable(segmenter src/plane_det/polygonization.cpp)

rosbuild_add_executable(pclDetector src/plane_det/pclDetector.cpp)
rosbuild_add_executable(pclDetector src/plane_det/dynModelExporter.cpp)
rosbuild_add_executable(pclDetector src/plane_det/normals.cpp)

rosbuild_add_executable(detector src/plane_det/detector.cpp)
rosbuild_add_executable(detector src/plane_det/normals.cpp)
rosbuild_add_executable(detector src/plane_det/filtering.cpp)
rosbuild_add_executable(detector src/plane_det/parameterSpace.cpp)
rosbuild_add_executable(detector src/plane_det/parameterSpaceOctomap.cpp)
rosbuild_add_executable(detector src/plane_det/parameterSpaceHierarchy.cpp)
rosbuild_add_executable(detector src/plane_det/sceneModel.cpp)
rosbuild_add_executable(detector src/plane_det/dynModelExporter.cpp)

rosbuild_add_executable(kin2pcl src/plane_det/kin2pcl.cpp)
rosbuild_add_executable(kin2pcl src/plane_det/normals.cpp)
rosbuild_add_executable(kin2pcl src/plane_det/parameterSpace.cpp)

rosbuild_add_executable(exporter src/plane_det/exporter.cpp)
rosbuild_add_executable(exporter src/plane_det/normals.cpp)
rosbuild_add_executable(exporter src/plane_det/parameterSpace.cpp)

# Bounding box estimator
rosbuild_add_executable(bb_estimator_server src/bb_estimator/bb_estimator_server.cpp)
rosbuild_add_executable(bb_estimator_client src/bb_estimator/bb_estimator_client.cpp)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

