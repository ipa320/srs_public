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

if(NOT EXISTS ${PROJECT_SOURCE_DIR}/lib/javaGeom-0.10.2.jar)
MESSAGE(STATUS " --> DOWNLOAD JAVAGEOM")
#rosbuild_download_data(http://sourceforge.net/projects/geom-java/files/javaGeom/javaGeom%200.10.2/javaGeom-0.10.2.jar/download lib/javaGeom-0.10.2.jar 6c1305d6c19fad12a6a935810d2aa46f)
file(DOWNLOAD http://sourceforge.net/projects/geom-java/files/javaGeom/javaGeom%200.10.2/javaGeom-0.10.2.jar/download ${PROJECT_SOURCE_DIR}/lib/javaGeom-0.10.2.jar SHOW_PROGRESS)
endif(NOT EXISTS ${PROJECT_SOURCE_DIR}/lib/javaGeom-0.10.2.jar)

add_jar_dir(${PROJECT_SOURCE_DIR}/lib)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

rosbuild_find_ros_package(srs_knowledge)

include(cmake/utils.cmake)
set(OWL_PATH_PREFIX ${srs_knowledge_PACKAGE_PATH})
fix_ros_destination_path(OWL_PATH_PREFIX)

file(GLOB inputs conf/ontologies/ipa-kitchen-map/*.in conf/ontologies/empty-map/*.in conf/ontologies/ipa-kitchen/*.in conf/ontologies/milan-site/*.in conf/ontologies/srs-milano2012/*.in conf/*.in)

foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

rospack_add_java_executable(bin/knowledgeEngine  org.srs.srs_knowledge.knowledge_engine.KnowledgeEngine)

