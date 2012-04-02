/******************************************************************************
 * \file
 *
 * $Id: init.cpp 396 2012-03-29 12:24:03Z spanel $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd/mm/2011
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rviz/plugin/type_registry.h"

#include "but_display.h"
#include "but_pointcloud.h"
#include "but_distance_visualizer.h"
#include "but_distance_indicator.h"
#include "but_data_fusion/but_cam_display.h"
#include "but_camcast.h"

extern "C" void rvizPluginInit(rviz::TypeRegistry* reg)
{
  reg->registerDisplay<CButDisplay> ("CButDisplay");
  reg->registerDisplay<rviz::CButPointCloud> ("CButPointCloud");
  reg->registerDisplay<rviz::CButDistanceVisualizer> ("CButDistanceVisualizer");
  reg->registerDisplay<rviz::CButDistanceIndicator> ("CButDistanceIndicator");
  reg->registerDisplay<rviz::CButCamDisplay> ("CButCamDisplay");
  reg->registerDisplay<CButCamCast> ("CButCamCast");
}

