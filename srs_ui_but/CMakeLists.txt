cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)


rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()

#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

rosbuild_add_boost_directories()
#rosbuild_link_boost( thread)

#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


# BUT services
set( BUT_SERVICES_SOURCE_FILES src/but_services/point_cloud_tools.cpp
                               src/but_services/service_server.cpp )
                      
set( BUT_SERVICES_PROJECT_NAME but_gui_services )
rosbuild_add_library( ${BUT_SERVICES_PROJECT_NAME} ${BUT_SERVICES_SOURCE_FILES} )

rosbuild_add_executable(but_gui_service_server src/but_services/service_server.cpp)
rosbuild_add_executable(cob_stretch_publisher src/but_services/cob_stretch_publisher.cpp)    
target_link_libraries(but_gui_service_server ${BUT_SERVICES_PROJECT_NAME})
#target_link_libraries(cob_stretch_publisher ${BUT_SERVICES_PROJECT_NAME})            


# BUT rviz display
set( DISPLAY_SOURCE_FILES src/but_display/but_display.cpp
                          src/but_display/but_point_cloud.cpp
                          src/but_display/point_cloud_base.cpp
                          src/but_display/example_pane.cpp
                          src/but_display/octomap_control_pane.cpp
                          src/but_display/but_distance_linear_visualizer.cpp
                          src/but_display/but_distance_circular_indicator.cpp
                          src/but_display/but_cob_stretch_indicator.cpp
                          src/but_data_fusion/but_cam_display.cpp
                          src/but_display/but_camcast.cpp
                          src/but_display/ros_rtt_texture.cpp
                          src/but_display/but_projection.cpp
						  src/but_display/but_rostexture.cpp
						  src/but_display/but_context_manager.cpp
						  src/but_display/but_object_manager.cpp
						  src/but_display/object_control_pane.cpp
						  src/but_display/but_depthtexture.cpp
                          src/but_display/init.cpp 
                          )

set( BUT_DISPLAY_PROJECT_NAME but_gui )
include_directories(include/srs_ui_but/but_display include/srs_ui_but/but_data_fusion srv_gen/cpp/include/srs_ui_but/)
rosbuild_add_library( ${BUT_DISPLAY_PROJECT_NAME} ${DISPLAY_SOURCE_FILES} )

# Add wx widgets
find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

target_link_libraries(${BUT_DISPLAY_PROJECT_NAME} ${wxWidgets_LIBRARIES} ${OGRE_LIBRARIES} )
rosbuild_link_boost(${BUT_DISPLAY_PROJECT_NAME} thread)

# Add but_depthtoimage_node executable
rosbuild_add_executable(but_depthtoimage_node src/but_display/but_depthpctoimage.cpp )


# BUT data fusion
rosbuild_add_executable(data_fusion_view src/but_data_fusion/view.cpp)

