/**
 * $Id: init.cpp 134 2012-01-12 13:52:36Z spanel $
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

extern "C" void rvizPluginInit(rviz::TypeRegistry* reg)
{
  reg->registerDisplay<CButDisplay>("BUT_Display");
  reg->registerDisplay<rviz::CButPointCloud>("CButPointCloud");
}


