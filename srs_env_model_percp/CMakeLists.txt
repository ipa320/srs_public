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


# TODO: Link against official BUT packages!
rosbuild_add_library( but_segmentation src/but_segmentation/filtering.cpp 
                                       src/but_segmentation/normals.cpp )
rosbuild_add_openmp_flags( but_segmentation )
include_directories( include/srs_env_model_percp )

# Plane detection node
set( BUT_PLANE_DETECTOR_SOURCES src/but_plane_detector/plane_detector_node.cpp 
                                src/but_plane_detector/parameter_space.cpp 
                                src/but_plane_detector/parameter_space_hierarchy.cpp 
                                src/but_plane_detector/scene_model.cpp 
                                src/but_plane_detector/dyn_model_exporter.cpp )
rosbuild_add_executable( but_plane_detector ${BUT_PLANE_DETECTOR_SOURCES} )
target_link_libraries( but_plane_detector but_segmentation )

# Demo plane detection node
set( BUT_SRS2_SOURCES src/but_plane_detector/plane_detector_demo_node.cpp 
                      src/but_plane_detector/parameter_space.cpp 
                      src/but_plane_detector/parameter_space_hierarchy.cpp 
                      src/but_plane_detector/scene_model.cpp 
                      src/but_plane_detector/dyn_model_exporter.cpp
                      src/but_plane_detector/dyn_model_exporter2.cpp )
rosbuild_add_executable( but_plane_detector_demo ${BUT_SRS2_SOURCES} )
target_link_libraries( but_plane_detector_demo but_segmentation )


# Plane detection node using pcl plane fitting
set( BUT_PLANE_DETECTOR_RANSAC_SOURCES src/but_plane_detector/plane_detector_ransac_node.cpp
                                src/but_plane_detector/parameter_space.cpp 
                                src/but_plane_detector/parameter_space_hierarchy.cpp 
                                src/but_plane_detector/scene_model.cpp 
                                src/but_plane_detector/dyn_model_exporter.cpp )
rosbuild_add_executable( but_plane_detector_ransac ${BUT_PLANE_DETECTOR_RANSAC_SOURCES} )
target_link_libraries( but_plane_detector_ransac but_segmentation )


# Kinect depth map to pcl converter node
rosbuild_add_executable(but_kin2pcl src/but_seg_utils/kin2pcl_node.cpp)
target_link_libraries(but_kin2pcl but_segmentation)

# Kinect depth map to pcd exporter
rosbuild_add_executable(but_pcd_exporter src/but_seg_utils/pcd_exporter_node.cpp)
target_link_libraries(but_pcd_exporter but_segmentation)


# Bounding box estimator
rosbuild_add_executable(bb_estimator_server src/bb_estimator/service_server.cpp src/bb_estimator/funcs.cpp )
rosbuild_add_executable(bb_estimator_client src/bb_estimator/sample_client.cpp)


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

