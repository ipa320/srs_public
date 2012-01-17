/**
 * $Id: init.cpp 140 2012-01-13 08:43:34Z stancl $
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
  reg->registerDisplay<CButDisplay>("CButDisplay");
  reg->registerDisplay<rviz::CButPointCloud>("CButPointCloud");
}


