/*
 *******************************************************************************
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 19.1.2012
 *******************************************************************************
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
#include <but_gui/Object.h>
#include <but_gui/ObjectWithBoundingBox.h>
#include <but_gui/UnknownObject.h>
#include "services_list.h"
#include <math.h>
#include <map>
#include <string>


using namespace but_gui;
using namespace std;

namespace but_gui
{

/*
 *------------------------------------------------------------------------------
 *  Server variables
 *------------------------------------------------------------------------------
 */

map<string, Primitive*> primitives;

// Interactive Marker server
InteractiveMarkerServerPtr imServer;

/*
 *------------------------------------------------------------------------------
 *  Services
 *------------------------------------------------------------------------------
 */

/* Plane adding.
 *
 * @param req  Request of type AddPlane.
 * @param res  Response of type AddPlane.
 */
bool addPlane(AddPlane::Request &req, AddPlane::Response &res);
/* Billboard adding.
 *
 * @param req  Request of type AddBillboard.
 * @param res  Response of type AddBillboard.
 */
bool addBillboard(AddBillboard::Request &req, AddBillboard::Response &res);
/* Bounding Box adding.
 *
 * @param req  Request of type AddBoundingBox.
 * @param res  Response of type AddBoundingBox.
 */
bool addBoundingBox(AddBoundingBox::Request &req, AddBoundingBox::Response &res);
/* Unknown Object adding.
 *
 * @param req  Request of type AddUnknownObject.
 * @param res  Response of type AddUnknownObject.
 */
bool addObject(AddObject::Request &req, AddObject::Response &res);
/* Object adding.
 *
 * @param req  Request of type AddObject.
 * @param res  Response of type AddObject.
 */
bool addObjectWithBoundingBox(AddObjectWithBoundingBox::Request &req, AddObjectWithBoundingBox::Response &res);
/* Object with bounding box adding.
 *
 * @param req  Request of type ObjectWithBoundingBox.
 * @param res  Response of type ObjectWithBoundingBox.
 */
bool addUnknownObject(AddUnknownObject::Request &req, AddUnknownObject::Response &res);
/* Marker adding.
 *
 * @param req  Request of type AddMarker.
 * @param res  Response of type AddMarker.
 */
bool addMarker(AddMarker::Request &req, AddMarker::Response &res);
/* Object removing.
 *
 * @param req  Request of type RemovePrimitive.
 * @param res  Response of type RemovePrimitive.
 */
bool removePrimitive(RemovePrimitive::Request &req, RemovePrimitive::Response &res);
/* Set grapsing position
 *
 * @param req  Request of type SetGraspingPosition.
 * @param res  Response of type SetGraspingPosition.
 */
bool setGraspingPosition(SetGraspingPosition::Request &req, SetGraspingPosition::Response &res);
/* Remove grapsing position
 *
 * @param req  Request of type RemoveGraspingPosition.
 * @param res  Response of type RemoveGraspingPosition.
 */
bool removeGraspingPosition(RemoveGraspingPosition::Request &req, RemoveGraspingPosition::Response &res);
/* Change object's description..
 *
 * @param req  Request of type ChangeDescription.
 * @param res  Response of type ChangeDescription.
 */
bool changeDescription(ChangeDescription::Request &req, ChangeDescription::Response &res);
/* Change object's pose.
 *
 * @param req  Request of type ChangePose.
 * @param res  Response of type ChangePose.
 */
bool changePose(ChangePose::Request &req, ChangePose::Response &res);
/* Change object's scale.
 *
 * @param req  Request of type ChangeScale.
 * @param res  Response of type ChangeScale.
 */
bool changeScale(ChangeScale::Request &req, ChangeScale::Response &res);
/* Change object's color.
 *
 * @param req  Request of type ChangeColor.
 * @param res  Response of type ChangeColor.
 */
bool changeColor(ChangeColor::Request &req, ChangeColor::Response &res);
/* Change object's direction
 *
 * @param req  Request of type ChangeDirection.
 * @param res  Response of type ChangeDirection.
 */
bool changeDirection(ChangeDirection::Request &req, ChangeDirection::Response &res);
/* Change object's velocity.
 *
 * @param req  Request of type ChangeVelocity.
 * @param res  Response of type ChangeVelocity.
 */
bool changeVelocity(ChangeVelocity::Request &req, ChangeVelocity::Response &res);

/* Gets object's update topic
 *
 * @param req  Request of type GetUpdateTopic.
 * @param res  Response of type GetUpdateTopic.
 */
bool getUpdateTopic(GetUpdateTopic::Request &req, GetUpdateTopic::Response &res);

}

#endif /* BUT_GUI_SERVICE_SERVER_H_ */
