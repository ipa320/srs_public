/******************************************************************************
 * \file
 *
 * $Id: services_list.h 676 2012-04-19 18:32:07Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 05/03/2012
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
#ifndef SERVICES_LIST_H_
#define SERVICES_LIST_H_

#include <srs_interaction_primitives/AddMarker.h>
#include <srs_interaction_primitives/AddBoundingBox.h>
#include <srs_interaction_primitives/AddBillboard.h>
#include <srs_interaction_primitives/AddPlane.h>
#include <srs_interaction_primitives/AddPlanePolygon.h>
#include <srs_interaction_primitives/AddObject.h>
#include <srs_interaction_primitives/AddUnknownObject.h>
#include <srs_interaction_primitives/RemovePrimitive.h>
#include <srs_interaction_primitives/ChangeDescription.h>
#include <srs_interaction_primitives/ChangePose.h>
#include <srs_interaction_primitives/ChangeScale.h>
#include <srs_interaction_primitives/ChangeColor.h>
#include <srs_interaction_primitives/ChangeVelocity.h>
#include <srs_interaction_primitives/ChangeDirection.h>
#include <srs_interaction_primitives/GetUpdateTopic.h>
#include <srs_interaction_primitives/SetPreGraspPosition.h>
#include <srs_interaction_primitives/RemovePreGraspPosition.h>
#include <srs_interaction_primitives/GetAllPrimitivesNames.h>

#include <string>

namespace srs_interaction_primitives
{
    static const std::string PACKAGE_NAME_PREFIX  = "/interaction_primitives";

	/*
	 * Services for adding of primitives
	 */
    static const std::string AddBillboard_SRV = PACKAGE_NAME_PREFIX + std::string("/add_billboard");
    static const std::string AddBoundingBox_SRV = PACKAGE_NAME_PREFIX + std::string("/add_bounding_box");
    static const std::string AddPlane_SRV = PACKAGE_NAME_PREFIX + std::string("/add_plane");
    static const std::string AddPlanePolygon_SRV = PACKAGE_NAME_PREFIX + std::string("/add_plane_polygon");
    static const std::string AddObject_SRV = PACKAGE_NAME_PREFIX + std::string("/add_object");
    static const std::string AddUnknownObject_SRV = PACKAGE_NAME_PREFIX + std::string("/add_unknown_object");

	/*
	 * Services for removing of primitives
	 */
    static const std::string RemovePrimitive_SRV = PACKAGE_NAME_PREFIX + std::string("/remove_primitive");

	/*
	 * Services for updating primitive's properties
	 */
    static const std::string ChangeDescription_SRV = PACKAGE_NAME_PREFIX + std::string("/change_description");
    static const std::string ChangePose_SRV = PACKAGE_NAME_PREFIX + std::string("/change_pose");
    static const std::string ChangeScale_SRV = PACKAGE_NAME_PREFIX + std::string("/change_scale");
    static const std::string ChangeColor_SRV = PACKAGE_NAME_PREFIX + std::string("/change_color");
    static const std::string ChangeDirection_SRV = PACKAGE_NAME_PREFIX + std::string("/change_direction");
    static const std::string ChangeVelocity_SRV = PACKAGE_NAME_PREFIX + std::string("/change_velocity");

	/*
	 * Services for setting and removing primitive's grasping positions
	 */
    static const std::string SetGraspingPosition_SRV = PACKAGE_NAME_PREFIX + std::string("/set_pregrasp_position");
    static const std::string RemoveGraspingPosition_SRV = PACKAGE_NAME_PREFIX + std::string("/remove_pregrasp_position");

	/*
	 * Services for update topics
	 */
    static const std::string GetUpdateTopic_SRV = PACKAGE_NAME_PREFIX + std::string("/get_update_topic");

	/*
	 * Other services
	 */
    static const std::string GetAllPrimitivesNames_SRV = PACKAGE_NAME_PREFIX + std::string("/get_all_primitives_names");
}

#endif /* SERVICES_LIST_H_ */
