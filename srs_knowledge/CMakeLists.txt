cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()

# find needed paths
# rosbuild_find_ros_package(srs_jena)
rosbuild_find_ros_package(srs_pellet)

#add_jar_dir(${srs_jena_PACKAGE_PATH}/common/lib)
add_jar_dir(${srs_pellet_PACKAGE_PATH}/common/lib)
add_jar_dir(${srs_pellet_PACKAGE_PATH}/common/lib/jena)
#add_jar_dir(/usr/share/java)
add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

rosbuild_find_ros_package(srs_knowledge)

# rosbuild_find_ros_package(ias_knowledge_base)
#rosbuild_include(rosprolog utils)
include(cmake/utils.cmake)
set(OWL_PATH_PREFIX ${srs_knowledge_PACKAGE_PATH})
fix_ros_destination_path(OWL_PATH_PREFIX)

file(GLOB inputs conf/ontologies/ipa-kitchen-map/*.in conf/ontologies/empty-map/*.in)

foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

rospack_add_java_executable(bin/knowledgeEngine  org.srs.srs_knowledge.knowledge_engine.KnowledgeEngine)
# rosbuild_add_rostest(test/rosjava_test.launch)

