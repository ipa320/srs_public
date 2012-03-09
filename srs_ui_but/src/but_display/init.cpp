/**
 * $Id: init.cpp 238 2012-02-21 15:39:29Z xblaho02 $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: dd.mm.2011
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 */

#include "rviz/plugin/type_registry.h"

#include "but_display.h"
#include "but_pointcloud.h"
#include "but_distance_visualizer.h"
#include "but_data_fusion/but_cam_display.h"

extern "C" void rvizPluginInit(rviz::TypeRegistry* reg)
{
  reg->registerDisplay<CButDisplay>("CButDisplay");
  reg->registerDisplay<rviz::CButPointCloud>("CButPointCloud");
  reg->registerDisplay<rviz::CButDistanceVisualizer>("CButDistanceVisualizer");
  reg->registerDisplay<rviz::ButCamDisplay>("ButCamDisplay");
}


