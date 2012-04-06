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

#define BUT_DATA_FUSION_PREFIX std::string("/but_data_fusion")
#define BUT_DATA_FUSION_TOPIC(topic) BUT_DATA_FUSION_PREFIX + std::string(topic)

// published topics
#define BUT_VIEW_FRUSTUM_TOP BUT_DATA_FUSION_TOPIC("/view_frustum")
#define BUT_CAMERA_VIEW_TOP BUT_DATA_FUSION_TOPIC("/cam_view")

// global parameters
#define BUT_CAMERA_PAR BUT_DATA_FUSION_TOPIC("/camera")
#define BUT_DEPTH_PAR BUT_DATA_FUSION_TOPIC("/depth")
