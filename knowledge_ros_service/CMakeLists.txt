cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()

# add_jar_dir("/home/ze/srs/coding/library/Jena-2.6.4/lib")
# add_jar_dir("/Users/ze/Downloads/Jena-2.6.4/lib")
#add_jar_dir($ENV{JENA_LIBS})
add_jar_dir($ENV{JENAROOT}/lib)
#add_jar_dir(/usr/share/java)
add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

#rospack_add_java_executable(bin/KnowledgeEngine KnowledgeEngine)
rospack_add_java_executable(bin/org.srs.knowledge_ros_service.knowledge_engine.KnowledgeEngine  org.srs.knowledge_ros_service.knowledge_engine.KnowledgeEngine)
# rosbuild_add_rostest(test/rosjava_test.launch)
