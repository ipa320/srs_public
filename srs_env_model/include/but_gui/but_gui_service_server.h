/******************************************************************************
 * \file
 *
 * $Id: but_gui_service_server.h 603 2012-04-16 10:50:03Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd.mm.2012
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

#ifndef BUT_GUI_SERVICE_SERVER_H_
#define BUT_GUI_SERVICE_SERVER_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <srs_env_model/PrimitiveType.h>
#include <but_gui/Primitive.h>
#include <but_gui/BoundingBox.h>
#include <but_gui/Billboard.h>
#include <but_gui/Plane.h>
#include <but_gui/PlanePolygon.h>
#include <but_gui/Object.h>
#include <but_gui/ObjectWithBoundingBox.h>
#include <but_gui/UnknownObject.h>
#include "services_list.h"
#include <math.h>
#include <map>
#include <string>

using namespace but_gui;
using namespace std;

/**
 * This server advertises services for but_gui.
 *
 * @author Tomas Lokaj
 */

namespace but_gui
{

// Container with primitives
map<string, Primitive*> primitives;

// Interactive Marker server
InteractiveMarkerServerPtr imServer;

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
 * @brief Unknown Object adding.
 *
 * @param req  Request of type AddUnknownObject.
 * @param res  Response of type AddUnknownObject.
 */
bool addObject(AddObject::Request &req, AddObject::Response &res);

/**
 * @brief Object adding.
 *
 * @param req  Request of type AddObject.
 * @param res  Response of type AddObject.
 */
bool addObjectWithBoundingBox(AddObjectWithBoundingBox::Request &req, AddObjectWithBoundingBox::Response &res);

/**
 * @brief Object with bounding box adding.
 *
 * @param req  Request of type ObjectWithBoundingBox.
 * @param res  Response of type ObjectWithBoundingBox.
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
 * @brief Set grapsing position
 *
 * @param req  Request of type SetGraspingPosition.
 * @param res  Response of type SetGraspingPosition.
 */
bool setGraspingPosition(SetGraspingPosition::Request &req, SetGraspingPosition::Response &res);

/**
 * @brief Remove grapsing position
 *
 * @param req  Request of type RemoveGraspingPosition.
 * @param res  Response of type RemoveGraspingPosition.
 */
bool removeGraspingPosition(RemoveGraspingPosition::Request &req, RemoveGraspingPosition::Response &res);

/**
 * @brief Change object's description.
 *
 * @param req  Request of type ChangeDescription.
 * @param res  Response of type ChangeDescription.
 */
bool changeDescription(ChangeDescription::Request &req, ChangeDescription::Response &res);

/**
 * @brief Change object's pose.
 *
 * @param req  Request of type ChangePose.
 * @param res  Response of type ChangePose.
 */
bool changePose(ChangePose::Request &req, ChangePose::Response &res);

/**
 * @brief Change object's scale.
 *
 * @param req  Request of type ChangeScale.
 * @param res  Response of type ChangeScale.
 */
bool changeScale(ChangeScale::Request &req, ChangeScale::Response &res);

/**
 * @brief Change object's color.
 *
 * @param req  Request of type ChangeColor.
 * @param res  Response of type ChangeColor.
 */
bool changeColor(ChangeColor::Request &req, ChangeColor::Response &res);

/**
 * @brief Change object's direction
 *
 * @param req  Request of type ChangeDirection.
 * @param res  Response of type ChangeDirection.
 */
bool changeDirection(ChangeDirection::Request &req, ChangeDirection::Response &res);

/**
 * @brief Change object's velocity.
 *
 * @param req  Request of type ChangeVelocity.
 * @param res  Response of type ChangeVelocity.
 */
bool changeVelocity(ChangeVelocity::Request &req, ChangeVelocity::Response &res);

/**
 * @brief Gets object's update topic
 *
 * @param req  Request of type GetUpdateTopic.
 * @param res  Response of type GetUpdateTopic.
 */
bool getUpdateTopic(GetUpdateTopic::Request &req, GetUpdateTopic::Response &res);


}

#endif /* BUT_GUI_SERVICE_SERVER_H_ */
