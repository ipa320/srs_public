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
set( GUI_SOURCE_FILES src/but_gui/BoundingBox.cpp
                      src/but_gui/GuiObject.cpp
                      src/but_gui/Billboard.cpp
                      src/but_gui/Plane.cpp
                      src/but_gui/UnknownObject.cpp )

set( BUT_GUI_PROJECT_NAME but_gui )
rosbuild_add_library( ${BUT_GUI_PROJECT_NAME} ${GUI_SOURCE_FILES} )

rosbuild_add_executable(but_gui_objects src/but_gui/but_gui_objects.cpp)
rosbuild_add_executable(but_gui_services src/but_gui/but_gui_services.cpp)
rosbuild_add_executable(but_gui_client src/but_gui/but_gui_client.cpp)

target_link_libraries(but_gui_objects ${BUT_GUI_PROJECT_NAME})
target_link_libraries(but_gui_services ${BUT_GUI_PROJECT_NAME})
target_link_libraries(but_gui_client ${BUT_GUI_PROJECT_NAME})

set( SERVER_SOURCES src/but_server/servernode.cpp
                    src/but_server/but_server.cpp
                    )

# Add servernode executable
set( BUT_DYNMODEL_PROJECT_NAME but_dynmodel )
rosbuild_add_executable(but_server_node ${SERVER_SOURCES} )

target_link_libraries(but_server_node ${OCTOMAP_LIBRARIES} ${BUT_GUI_PROJECT_NAME} )

