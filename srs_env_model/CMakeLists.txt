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

# BUT context server
set( CONTEXT_SOURCE_FILES src/but_context/ContextServer.cpp)

set( BUT_CONTEXT_PROJECT_NAME but_context )
rosbuild_add_library( ${BUT_CONTEXT_PROJECT_NAME} ${CONTEXT_SOURCE_FILES} )

rosbuild_add_executable(but_context_server src/but_context/but_context_server.cpp)

target_link_libraries(but_context_server ${BUT_CONTEXT_PROJECT_NAME})

# BUT gui primitives
set( GUI_SOURCE_FILES src/but_gui/BoundingBox.cpp
                      src/but_gui/Primitive.cpp
                      src/but_gui/Billboard.cpp
                      src/but_gui/Plane.cpp
                      src/but_gui/PlanePolygon.cpp
                      src/but_gui/Object.cpp
                      src/but_gui/ObjectWithBoundingBox.cpp
                      src/but_gui/UnknownObject.cpp
                      src/but_gui/UpdatePublisher.cpp
                      src/but_gui/but_gui_service_server.cpp  )

set( BUT_GUI_PROJECT_NAME but_gui )
rosbuild_add_library( ${BUT_GUI_PROJECT_NAME} ${GUI_SOURCE_FILES} )

rosbuild_add_executable(but_gui_service_server src/but_gui/but_gui_service_server.cpp)
rosbuild_add_executable(but_gui_test_primitives src/but_gui/test_primitives.cpp)
rosbuild_add_executable(but_gui_test_observers src/but_gui/test_observers.cpp)

target_link_libraries(but_gui_service_server ${BUT_GUI_PROJECT_NAME})
target_link_libraries(but_gui_test_primitives ${BUT_GUI_PROJECT_NAME})
target_link_libraries(but_gui_test_observers ${BUT_GUI_PROJECT_NAME})

set( SERVER_SOURCES src/but_server/servernode.cpp
                    src/but_server/but_server.cpp
                    src/but_server/ServerTools.cpp
                    src/but_server/octonode.cpp
                    src/but_server/plugins/CMapPlugin.cpp
                    src/but_server/plugins/OctoMapPlugin.cpp
                    src/but_server/plugins/PointCloudPlugin.cpp
                    src/but_server/plugins/CollisionObjectPlugin.cpp
                    src/but_server/plugins/Map2DPlugin.cpp
                    src/but_server/plugins/IMarkersPlugin.cpp
                    src/but_server/plugins/MarkerArrayPlugin.cpp
                    src/but_server/plugins/LimitedPointCloudPlugin.cpp
                    src/but_server/plugins/OldIMarkersPlugin.cpp
                    )

# Add servernode executable
set( BUT_DYNMODEL_PROJECT_NAME but_dynmodel )
rosbuild_add_executable(but_server_node ${SERVER_SOURCES} )

target_link_libraries(but_server_node ${OCTOMAP_LIBRARIES} ${BUT_GUI_PROJECT_NAME} )

