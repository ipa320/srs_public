/******************************************************************************
 * \file
 *
 * $Id: interaction_primitives_service_server.h 676 2012-04-19 18:32:07Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 9/12/2011
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
#ifndef INTERACTION_PRIMITIVES_SERVICE_SERVER_H_
#define INTERACTION_PRIMITIVES_SERVICE_SERVER_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

#include <srs_interaction_primitives/PrimitiveType.h>
#include <std_msgs/UInt8.h>

#include "primitive.h"
#include "bounding_box.h"
#include "billboard.h"
#include "plane.h"
#include "plane_polygon.h"
#include "object.h"
#include "unknown_object.h"
#include "clickable_positions/clickable_positions.h"
#include "services_list.h"

#include <cmath>
#include <map>
#include <string>

/**
 * This server advertises services for interactive primitives.
 *
 * @author Tomas Lokaj
 */

namespace srs_interaction_primitives
{

// Container with primitives
extern std::map<std::string, Primitive*> primitives;

// Interactive Marker server
extern InteractiveMarkerServerPtr imServer;

ros::Publisher vis_pub;

/**
 * @brief  Plane adding.
 *
 * @param req  Request of type AddPlane.
 * @param res  Response of type AddPlane.
 */
bool addPlane(AddPlane::Request &req, AddPlane::Response &res);

/**
 * @brief  PlanePolygon adding.
 *
 * @param req  Request of type AddPlanePolygon.
 * @param res  Response of type AddPlanePolygon.
 */
bool addPlanePolygon(AddPlanePolygon::Request &req, AddPlanePolygon::Response &res);

/**
 * @brief Billboard adding.
 *
 * @param req  Request of type AddBillboard.
 * @param res  Response of type AddBillboard.
 */
bool addBillboard(AddBillboard::Request &req, AddBillboard::Response &res);

/**
 * @brief Bounding Box adding.
 *
 * @param req  Request of type AddBoundingBox.
 * @param res  Response of type AddBoundingBox.
 */
bool addBoundingBox(AddBoundingBox::Request &req, AddBoundingBox::Response &res);

/**
 * @brief  Object adding.
 *
 * @param req  Request of type AddObject.
 * @param res  Response of type AddObject.
 */
bool addObject(AddObject::Request &req, AddObject::Response &res);

/**
 * @brief Unknown Object adding.
 *
 * @param req  Request of type AddUnknownObject.
 * @param res  Response of type AddUnknownObject.
 */
bool addUnknownObject(AddUnknownObject::Request &req, AddUnknownObject::Response &res);

/**
 * @brief Object removing.
 *
 * @param req  Request of type RemovePrimitive.
 * @param res  Response of type RemovePrimitive.
 */
bool removePrimitive(RemovePrimitive::Request &req, RemovePrimitive::Response &res);

/**
 * @brief Set pre-grasp position
 *
 * @param req  Request of type SetPreGraspPosition.
 * @param res  Response of type SetPreGraspPosition.
 */
bool setPreGraspPosition(SetPreGraspPosition::Request &req, SetPreGraspPosition::Response &res);

/**
 * @brief Remove pre-grasp position
 *
 * @param req  Request of type RemoveGraspingPosition.
 * @param res  Response of type RemoveGraspingPosition.
 */
bool removePreGraspPosition(RemovePreGraspPosition::Request &req, RemovePreGraspPosition::Response &res);

/**
 * @brief Change primitive's description.
 *
 * @param req  Request of type ChangeDescription.
 * @param res  Response of type ChangeDescription.
 */
bool changeDescription(ChangeDescription::Request &req, ChangeDescription::Response &res);

/**
 * @brief Change primitive's pose.
 *
 * @param req  Request of type ChangePose.
 * @param res  Response of type ChangePose.
 */
bool changePose(ChangePose::Request &req, ChangePose::Response &res);

/**
 * @brief Change primitive's scale.
 *
 * @param req  Request of type ChangeScale.
 * @param res  Response of type ChangeScale.
 */
bool changeScale(ChangeScale::Request &req, ChangeScale::Response &res);

/**
 * @brief Change primitive's color.
 *
 * @param req  Request of type ChangeColor.
 * @param res  Response of type ChangeColor.
 */
bool changeColor(ChangeColor::Request &req, ChangeColor::Response &res);

/**
 * @brief Change primitive's direction
 *
 * @param req  Request of type ChangeDirection.
 * @param res  Response of type ChangeDirection.
 */
bool changeDirection(ChangeDirection::Request &req, ChangeDirection::Response &res);

/**
 * @brief Change primitive's velocity.
 *
 * @param req  Request of type ChangeVelocity.
 * @param res  Response of type ChangeVelocity.
 */
bool changeVelocity(ChangeVelocity::Request &req, ChangeVelocity::Response &res);

/**
 * @brief Gets primitive's update topic
 *
 * @param req  Request of type GetUpdateTopic.
 * @param res  Response of type GetUpdateTopic.
 */
bool getUpdateTopic(GetUpdateTopic::Request &req, GetUpdateTopic::Response &res);

/**
 * @brief Allows or denies interaction with Object
 *
 * @param req  Request of type SetAllowObjectInteraction.
 * @param res  Response of type SetAllowObjectInteraction.
 */
bool setAllowObjectInteraction(SetAllowObjectInteraction::Request &req, SetAllowObjectInteraction::Response &res);

/**
 * @brief Gets object
 *
 * @param req  Request of type GetObject.
 * @param res  Response of type GetObject.
 */
bool getObject(GetObject::Request &req, GetObject::Response &res);

/**
 * @brief Gets clicked position
 *
 * @param req  Request of type ClickPositions.
 * @param res  Response of type ClickPositions.
 */
bool clickablePositions(ClickablePositions::Request &req, ClickablePositions::Response &res);

/**
 * @brief Show predicted robot's positions and trajectory
 *
 * @param req  Request of type RobotPosePrediction.
 * @param res  Response of type RobotPosePrediction.
 */
bool robotPosePrediction(RobotPosePrediction::Request &req, RobotPosePrediction::Response &res);

}

#endif /* INTERACTION_PRIMITIVES_SERVICE_SERVER_H_ */
