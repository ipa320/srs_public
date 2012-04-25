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
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# BUT gui primitives
set( INTERACTION_PRIMITIVES_SOURCE_FILES 
     src/but_interaction_primitives/BoundingBox.cpp
     src/but_interaction_primitives/Primitive.cpp
     src/but_interaction_primitives/Billboard.cpp
     src/but_interaction_primitives/Plane.cpp
     src/but_interaction_primitives/PlanePolygon.cpp
     src/but_interaction_primitives/Object.cpp
     src/but_interaction_primitives/UnknownObject.cpp
     src/but_interaction_primitives/UpdatePublisher.cpp )

set( INTERACTION_PRIMITIVES_LIB_NAME but_interaction_primitives )
rosbuild_add_library( ${INTERACTION_PRIMITIVES_LIB_NAME} ${INTERACTION_PRIMITIVES_SOURCE_FILES} )

rosbuild_add_executable( but_interaction_primitives_service_server src/but_interaction_primitives/service_server.cpp )
rosbuild_add_executable( but_interaction_primitives_test_primitives src/test_primitives.cpp )
rosbuild_add_executable( but_interaction_primitives_test_observers src/test_observers.cpp )

target_link_libraries( but_interaction_primitives_service_server ${INTERACTION_PRIMITIVES_LIB_NAME} )
target_link_libraries( but_interaction_primitives_test_primitives ${INTERACTION_PRIMITIVES_LIB_NAME} )
target_link_libraries( but_interaction_primitives_test_observers ${INTERACTION_PRIMITIVES_LIB_NAME} )

