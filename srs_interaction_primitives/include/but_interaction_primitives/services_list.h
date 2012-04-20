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

#define INTERACTION_PRIMITIVES_PREFIX std::string("/but_interaction_primitives")
#define INTERACTION_PRIMITIVES_SERVICE_TOPIC(topic) INTERACTION_PRIMITIVES_PREFIX + std::string(topic)

/*
 * Services for adding of primitives
 */
#define BUT_AddBillboard_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/add_billboard")
#define BUT_AddBoundingBox_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/add_bounding_box")
#define BUT_AddPlane_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/add_plane")
#define BUT_AddPlanePolygon_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/add_plane_polygon")
#define BUT_AddObject_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/add_object")
#define BUT_AddUnknownObject_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/add_unknown_object")

/*
 * Services for removing of primitives
 */
#define BUT_RemovePrimitive_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/remove_primitive")

/*
 * Services for updating primitive's properties
 */
#define BUT_ChangeDescription_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/change_description")
#define BUT_ChangePose_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/change_pose")
#define BUT_ChangeScale_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/change_scale")
#define BUT_ChangeColor_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/change_color")
#define BUT_ChangeDirection_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/change_direction")
#define BUT_ChangeVelocity_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/change_velocity")

/*
 * Services for setting and removing primitive's grasping positions
 */
#define BUT_SetGraspingPosition_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/set_pregrasp_position")
#define BUT_RemoveGraspingPosition_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/remove_pregrasp_position")

/*
 * Services for update topics
 */
#define BUT_GetUpdateTopic_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/get_update_topic")

/*
 * Other services
 */
#define BUT_GetAllPrimitivesNames_SRV INTERACTION_PRIMITIVES_SERVICE_TOPIC("/get_all_primitives_names")
#endif /* SERVICES_LIST_H_ */
