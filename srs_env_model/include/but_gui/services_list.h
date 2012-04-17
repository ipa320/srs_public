/******************************************************************************
 * \file
 *
 * $Id: services_list.h 603 2012-04-16 10:50:03Z xlokaj03 $
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

#include <srs_env_model/AddMarker.h>
#include <srs_env_model/AddBoundingBox.h>
#include <srs_env_model/AddBillboard.h>
#include <srs_env_model/AddPlane.h>
#include <srs_env_model/AddPlanePolygon.h>
#include <srs_env_model/AddObject.h>
#include <srs_env_model/AddObjectWithBoundingBox.h>
#include <srs_env_model/AddUnknownObject.h>
#include <srs_env_model/RemovePrimitive.h>
#include <srs_env_model/ChangeDescription.h>
#include <srs_env_model/ChangePose.h>
#include <srs_env_model/ChangeScale.h>
#include <srs_env_model/ChangeColor.h>
#include <srs_env_model/ChangeVelocity.h>
#include <srs_env_model/ChangeDirection.h>
#include <srs_env_model/GetUpdateTopic.h>
#include <srs_env_model/SetGraspingPosition.h>
#include <srs_env_model/RemoveGraspingPosition.h>

#define BUT_GUI_PREFIX std::string("/but_gui")
#define BUT_GUI_SERVICE_TOPIC(topic) BUT_GUI_PREFIX + std::string(topic)

/*
 * Services for adding of primitives
 */
#define BUT_AddBillboard_SRV BUT_GUI_SERVICE_TOPIC("/add_billboard")
#define BUT_AddBoundingBox_SRV BUT_GUI_SERVICE_TOPIC("/add_bounding_box")
#define BUT_AddPlane_SRV BUT_GUI_SERVICE_TOPIC("/add_plane")
#define BUT_AddPlanePolygon_SRV BUT_GUI_SERVICE_TOPIC("/add_plane_polygon")
#define BUT_AddObject_SRV BUT_GUI_SERVICE_TOPIC("/add_object")
#define BUT_AddUnknownObject_SRV BUT_GUI_SERVICE_TOPIC("/add_unknown_object")
#define BUT_AddObjectWithBoundingBox_SRV BUT_GUI_SERVICE_TOPIC("/add_object_with_bounding_box")

/*
 * Services for removing of primitives
 */
#define BUT_RemovePrimitive_SRV BUT_GUI_SERVICE_TOPIC("/remove_primitive")

/*
 * Services for updating primitive's properties
 */
#define BUT_ChangeDescription_SRV BUT_GUI_SERVICE_TOPIC("/change_description")
#define BUT_ChangePose_SRV BUT_GUI_SERVICE_TOPIC("/change_pose")
#define BUT_ChangeScale_SRV BUT_GUI_SERVICE_TOPIC("/change_scale")
#define BUT_ChangeColor_SRV BUT_GUI_SERVICE_TOPIC("/change_color")
#define BUT_ChangeDirection_SRV BUT_GUI_SERVICE_TOPIC("/change_direction")
#define BUT_ChangeVelocity_SRV BUT_GUI_SERVICE_TOPIC("/change_velocity")

/*
 * Services for setting and removing primitive's grasping positions
 */
#define BUT_SetGraspingPosition_SRV BUT_GUI_SERVICE_TOPIC("/set_pregrasp_position")
#define BUT_RemoveGraspingPosition_SRV BUT_GUI_SERVICE_TOPIC("/remove_pregrasp_position")

/*
 * Services for update topics
 */
#define BUT_GetUpdateTopic_SRV BUT_GUI_SERVICE_TOPIC("/get_update_topic")

#endif /* SERVICES_LIST_H_ */
