/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 3/12/2012
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

#include <srs_env_model_ui/but_rviz_display/cam_publisher_display.h>
#include <srs_env_model_ui/but_rviz_display/octomap_display.h>
#include <srs_env_model_ui/but_rviz_display/point_cloud_display.h>

extern "C" void rvizPluginInit(rviz::TypeRegistry* reg)
{
  reg->registerDisplay<srs_env_model_ui::CCameraPublisherDisplay>("CCameraPublisherDisplay");
  reg->registerDisplay<srs_env_model_ui::COctomapDisplay>("COctomapDisplay");
  reg->registerDisplay<srs_env_model_ui::CButPointCloud>("CButPointCloud");
}
