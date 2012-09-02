/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Vladimir Blahoz
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd/mm/2012
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

#pragma once
#ifndef BUT_GUI_TOPICS_LIST_H
#define BUT_GUI_TOPICS_LIST_H

#include "services_list.h"

namespace srs_ui_but
{
/**
 * but_data_fusion
 */
static const std::string BUT_DATA_FUSION_PREFIX = "/but_gui/data_fusion";
static const std::string BUT_SERVICES_PREFIX = "/but_gui/but_services";

/**
 * but_data_fusion - published topics
 */
static const std::string ViewFrustum_TOPIC = BUT_DATA_FUSION_PREFIX + std::string("/view_frustum");
static const std::string CameraView_TOPIC = BUT_DATA_FUSION_PREFIX + std::string("/cam_view");

/**
 * but_data_fusion - global parameters
 */
static const std::string Camera_PARAM = BUT_DATA_FUSION_PREFIX + std::string("/camera");
static const std::string Depth_PARAM = BUT_DATA_FUSION_PREFIX + std::string("/depth");

/**
 * but_data_fusion - remapped parameters
 */
static const std::string DEFAULT_CAMERA_INFO = std::string("default_camera_info");
static const std::string CAM3D_BASE = std::string("/cam3d/rgb/");
static const std::string STEREO_LEFT_BASE = std::string("/stereo/left/");
static const std::string STEREO_RIGHT_BASE = std::string("/stereo/right/");
static const std::string MAP_TOPIC = std::string("/map");
static const std::string DEPTH_IMAGE_IN = std::string("depth_image_in");
static const std::string DEFAULT_CAMERA_IMAGE = std::string("default_camera_image");


/**
 * but-display - parameters
 */
static const std::string DEFAULT_GRIPPER_LINK = "/sdh_palm_link";
static const std::string DEFAULT_ROBOT_LINK = "/sdh_palm_link";
static const std::string DEFAULT_COB_BASE_LINK = "/base_link";

/**
 * but-servcies - published topics
 */
static const std::string COBStretch_TOPIC = BUT_SERVICES_PREFIX + std::string("/cob_stretch");

/**
 * but-services - parameters
 */
//static const std::string CAMERA_TOPIC = "/cam3d/depth/points";
static const std::string CAMERA_TOPIC = "/but_env_model/pointcloud_centers";
static const std::string CAMERA_LINK = "/head_cam3d_link";
}

#endif // BUT_GUI_TOPICS_LIST_H




