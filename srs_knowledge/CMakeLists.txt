cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()

# find needed paths
rosbuild_find_ros_package(srs_jena)

add_jar_dir(${srs_jena_PACKAGE_PATH}/common/lib)
#add_jar_dir(/usr/share/java)
add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

rospack_add_java_executable(bin/knowledgeEngine  org.srs.srs_knowledge.knowledge_engine.KnowledgeEngine)
# rosbuild_add_rostest(test/rosjava_test.launch)
