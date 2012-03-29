/******************************************************************************
 * \file
 *
 * $Id: but_gui_service_server.cpp 397 2012-03-29 12:50:30Z spanel $
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

/**
 * Description:
 * This server advertises services for but_gui
 */

#include <but_gui/but_gui_service_server.h>

namespace but_gui
{
bool addPlane(AddPlane::Request &req, AddPlane::Response &res)
{
  ROS_INFO("ADDING PLANE");

  InteractiveMarker tmp;
  if ((imServer->get(req.name, tmp)) || (primitives.count(req.name) != 0))
  {
    ROS_ERROR("Object with that name already exists! Please remove it first.");
    return false;
  }

  Plane *plane = new Plane(imServer, req.frame_id, req.name);
  plane->setPose(req.pose);
  plane->setScale(req.scale);
  plane->setColor(req.color);
  plane->insert();
  imServer->applyChanges();

  primitives.insert(make_pair(req.name, plane));

  ROS_INFO("..... DONE");
  return true;
}

bool addBillboard(AddBillboard::Request &req, AddBillboard::Response &res)
{
  ROS_INFO("ADDING BILLBOARD");

  InteractiveMarker tmp;
  if ((imServer->get(req.name, tmp)) || (primitives.count(req.name) != 0))
  {
    ROS_ERROR("Object with that name already exists! Please remove it first.");
    return false;
  }

  Billboard *billboard = new Billboard(imServer, req.frame_id, req.name);
  billboard->setType(req.type);
  billboard->setPose(req.pose);
  billboard->setScale(req.scale);
  billboard->setDescription(req.description);
  billboard->setDirection(req.direction);
  billboard->setVelocity(req.velocity);
  billboard->insert();
  imServer->applyChanges();

  primitives.insert(make_pair(req.name, billboard));

  ROS_INFO("..... DONE");
  return true;
}

bool addBoundingBox(AddBoundingBox::Request &req, AddBoundingBox::Response &res)
{
  ROS_INFO("ADDING BOUNDING BOX");

  InteractiveMarker tmp;
  if ((imServer->get(req.name, tmp)) || (primitives.count(req.name) != 0))
  {
    ROS_ERROR("Object with that name already exists! Please remove it first.");
    return false;
  }

  BoundingBox * boundingBox = new BoundingBox(imServer, req.frame_id, req.name);
  boundingBox->setAttachedObjectName(req.object_name);
  boundingBox->setPose(req.pose);
  boundingBox->setScale(req.scale);
  boundingBox->setColor(req.color);
  boundingBox->setDescription(req.description);
  boundingBox->insert();
  imServer->applyChanges();

  primitives.insert(make_pair(req.name, boundingBox));

  ROS_INFO("..... DONE");
  return true;
}

bool addObject(AddObject::Request &req, AddObject::Response &res)
{
  ROS_INFO("ADDING OBJECT");

  InteractiveMarker tmp;
  if ((imServer->get(req.name, tmp)) || (primitives.count(req.name) != 0))
  {
    ROS_ERROR("Object with that name already exists! Please remove it first.");
    return false;
  }

  Object * object = new Object(imServer, req.frame_id, req.name);
  object->setResource(req.resource);
  object->setUseMaterial(req.use_material);
  object->setPose(req.pose);
  object->setScale(req.scale);
  object->setColor(req.color);
  object->setDescription(req.description);
  object->insert();
  imServer->applyChanges();

  primitives.insert(make_pair(req.name, object));

  ROS_INFO("..... DONE");
  return true;
}

bool addObjectWithBoundingBox(AddObjectWithBoundingBox::Request &req, AddObjectWithBoundingBox::Response &res)
{
  ROS_INFO("ADDING OBJECT WITH BOUNDING BOX");

  InteractiveMarker tmp;
  if ((imServer->get(req.name, tmp)) || (primitives.count(req.name) != 0))
  {
    ROS_ERROR("Object with that name already exists! Please remove it first.");
    return false;
  }

  ObjectWithBoundingBox * object = new ObjectWithBoundingBox(imServer, req.frame_id, req.name);
  if (req.resource == "")
  {
    ROS_INFO("SHAPE");
    object->setShape(req.shape);
    ROS_INFO("done");
  }
  else
  {
    ROS_INFO("RESOURCE");
    object->setResource(req.resource);
    object->setUseMaterial(req.use_material);
  }
//  object->setPose(req.pose);
  object->setPoseLWH(req.pose, req.bounding_box_lwh);
  object->setBoundingBoxLWH(req.bounding_box_lwh);
  object->setColor(req.color);
  object->setDescription(req.description);
  object->insert();
  imServer->applyChanges();

  primitives.insert(make_pair(req.name, object));

  ROS_INFO("..... DONE");
  return true;
}

bool addUnknownObject(AddUnknownObject::Request &req, AddUnknownObject::Response &res)
{
  ROS_INFO("ADDING UNKNOWN OBJECT");

  InteractiveMarker tmp;
  if ((imServer->get(req.name, tmp)) || (primitives.count(req.name) != 0))
  {
    ROS_ERROR("Object with that name already exists! Please remove it first.");
    return false;
  }

  UnknownObject * unknownObject = new UnknownObject(imServer, req.frame_id, req.name);
  unknownObject->setPose(req.pose);
  unknownObject->setScale(req.scale);
  unknownObject->setDescription(req.description);
  unknownObject->insert();
  imServer->applyChanges();

  primitives.insert(make_pair(req.name, unknownObject));

  ROS_INFO("..... DONE");
  return true;
}

bool addMarker(AddMarker::Request &req, AddMarker::Response &res)
{
  ROS_INFO("ADDING MARKER");

  InteractiveMarker tmp;
  if ((imServer->get(req.name, tmp)) || (primitives.count(req.name) != 0))
  {
    ROS_ERROR("Object with that name already exists! Please remove it first.");
    return false;
  }

  InteractiveMarker object;
  Marker marker;
  marker.type = req.type;
  marker.color = req.color;
  marker.scale = req.scale;
  object.header.frame_id = req.frame_id;
  object.header.stamp = ros::Time::now();
  object.name = req.name;
  object.description = req.description;
  object.pose = req.pose;
  InteractiveMarkerControl control;
  control.name = object.name + "_control";
  control.interaction_mode = InteractiveMarkerControl::NONE;
  control.always_visible = true;
  control.markers.push_back(marker);
  object.controls.push_back(control);
  imServer->insert(object);
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

bool removePrimitive(RemovePrimitive::Request &req, RemovePrimitive::Response &res)
{
  ROS_INFO("REMOVING PRIMITIVE");

  InteractiveMarker tmp;
  if (!imServer->get(req.name, tmp))
  {
    ROS_ERROR("Primitive with that name doesn't exist!");
    return false;
  }

  if (primitives.count(req.name) > 0)
    primitives.erase(primitives.find(req.name));

  imServer->erase(req.name);
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

bool setGraspingPosition(SetGraspingPosition::Request &req, SetGraspingPosition::Response &res)
{
  ROS_INFO("SETTING GRASPING POSITION");

  InteractiveMarker tmp;
  if ((!imServer->get(req.name, tmp)) || (primitives.count(req.name) != 1))
  {
    ROS_ERROR("Object with that name doesn't exist!");
    return false;
  }

  primitives[req.name]->setGraspingPosition(req.pos_id, req.position);
  primitives[req.name]->insert();
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

bool removeGraspingPosition(RemoveGraspingPosition::Request &req, RemoveGraspingPosition::Response &res)
{
  ROS_INFO("REMOVING GRASPING POSITION");

  InteractiveMarker tmp;
  if ((!imServer->get(req.name, tmp)) || (primitives.count(req.name) != 1))
  {
    ROS_ERROR("Object with that name doesn't exist!");
    return false;
  }

  primitives[req.name]->removeGraspingPosition(req.pos_id);
  primitives[req.name]->insert();
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

bool changeDescription(ChangeDescription::Request &req, ChangeDescription::Response &res)
{
  ROS_INFO("CHANGING DESCRIPTION");

  InteractiveMarker tmp;
  if ((!imServer->get(req.name, tmp)) || (primitives.count(req.name) != 1))
  {
    ROS_ERROR("Object with that name doesn't exist!");
    return false;
  }

  primitives[req.name]->setDescription(req.description);
  primitives[req.name]->insert();
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

bool changePose(ChangePose::Request &req, ChangePose::Response &res)
{
  ROS_INFO("CHANGING POSE");

  InteractiveMarker tmp;
  if ((!imServer->get(req.name, tmp)) || (primitives.count(req.name) != 1))
  {
    ROS_ERROR("Object with that name doesn't exist!");
    return false;
  }

  primitives[req.name]->setPose(req.pose);
  primitives[req.name]->insert();
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

bool changeScale(ChangeScale::Request &req, ChangeScale::Response &res)
{
  ROS_INFO("CHANGING SCALE");

  InteractiveMarker tmp;
  if ((!imServer->get(req.name, tmp)) || (primitives.count(req.name) != 1))
  {
    ROS_ERROR("Object with that name doesn't exist!");
    return false;
  }

  primitives[req.name]->setScale(req.scale);
  primitives[req.name]->insert();
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

bool changeColor(ChangeColor::Request &req, ChangeColor::Response &res)
{
  ROS_INFO("CHANGING COLOR");

  InteractiveMarker tmp;
  if ((!imServer->get(req.name, tmp)) || (primitives.count(req.name) != 1))
  {
    ROS_ERROR("Object with that name doesn't exist!");
    return false;
  }

  primitives[req.name]->setColor(req.color);
  primitives[req.name]->insert();
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

bool changeDirection(ChangeDirection::Request &req, ChangeDirection::Response &res)
{
  ROS_INFO("CHANGING DIRECTION");

  InteractiveMarker tmp;
  if ((!imServer->get(req.name, tmp)) || (primitives.count(req.name) != 1))
  {
    ROS_ERROR("Object with that name doesn't exist!");
    return false;
  }

  if (primitives[req.name]->getPrimitiveType() != PrimitiveType::BILLBOARD)
  {
    ROS_WARN("This is object is not a billboard, direction cannot be changed!");
    return false;
  }

  primitives[req.name]->setDirection(req.direction);
  primitives[req.name]->insert();
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

bool changeVelocity(ChangeVelocity::Request &req, ChangeVelocity::Response &res)
{
  ROS_INFO("CHANGING VELOCITY");

  InteractiveMarker tmp;
  if ((!imServer->get(req.name, tmp)) || (primitives.count(req.name) != 1))
  {
    ROS_ERROR("Object with that name doesn't exist!");
    return false;
  }

  if (primitives[req.name]->getPrimitiveType() != PrimitiveType::BILLBOARD)
  {
    ROS_WARN("This is object is not a billboard, velocity cannot be changed!");
    return false;
  }

  primitives[req.name]->setVelocity(req.velocity);
  primitives[req.name]->insert();
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

bool getUpdateTopic(GetUpdateTopic::Request &req, GetUpdateTopic::Response &res)
{
  ROS_INFO("GETTING UPDATE TOPIC");

  InteractiveMarker tmp;
  if ((!imServer->get(req.name, tmp)) || (primitives.count(req.name) != 1))
  {
    ROS_ERROR("Object with that name doesn't exist!");
    return false;
  }

  res.update_topic = primitives[req.name]->getUpdateTopic(req.type);

  ROS_INFO("..... DONE");
  return true;
}
}

int main(int argc, char **argv)
{
  /*
   * Main function
   */

  // ROS initialization (the last argument is the name of the node)
  ros::init(argc, argv, "but_gui_service_server");

  // Interactive Marker Server initialization
  imServer.reset(new InteractiveMarkerServer("but_gui", "", false));
  imServer->clear();

  // NodeHandle is the main access point to communications with the ROS system
  ros::NodeHandle n;

  // Create and advertise this service over ROS
  //ros::ServiceServer addMarkerService = n.advertiseService(BUT_AddBillboard_SRV, addMarker);
  ros::ServiceServer addBoundingBoxService = n.advertiseService(BUT_AddBoundingBox_SRV, addBoundingBox);
  ros::ServiceServer addBillboardService = n.advertiseService(BUT_AddBillboard_SRV, addBillboard);
  ros::ServiceServer addPlaneService = n.advertiseService(BUT_AddPlane_SRV, addPlane);
  ros::ServiceServer addObjectService = n.advertiseService(BUT_AddObject_SRV, addObject);
  ros::ServiceServer addObjectWithBoundingBoxService = n.advertiseService(BUT_AddObjectWithBoundingBox_SRV,
                                                                          addObjectWithBoundingBox);
  ros::ServiceServer addUnknownObjectService = n.advertiseService(BUT_AddUnknownObject_SRV, addUnknownObject);

  ros::ServiceServer removePrimitiveService = n.advertiseService(BUT_RemovePrimitive_SRV, removePrimitive);

  ros::ServiceServer changeDescriptionService = n.advertiseService(BUT_ChangeDescription_SRV, changeDescription);
  ros::ServiceServer changePoseService = n.advertiseService(BUT_ChangePose_SRV, changePose);
  ros::ServiceServer changeScaleService = n.advertiseService(BUT_ChangeScale_SRV, changeScale);
  ros::ServiceServer changeColorService = n.advertiseService(BUT_ChangeColor_SRV, changeColor);
  ros::ServiceServer changeDirectionService = n.advertiseService(BUT_ChangeDirection_SRV, changeDirection);
  ros::ServiceServer changeVelocityService = n.advertiseService(BUT_ChangeVelocity_SRV, changeVelocity);

  ros::ServiceServer setGraspingPositionService = n.advertiseService(BUT_SetGraspingPosition_SRV, setGraspingPosition);
  ros::ServiceServer removeGraspingPositionService = n.advertiseService(BUT_RemoveGraspingPosition_SRV,
                                                                        removeGraspingPosition);

  ros::ServiceServer getUpdateTopicService = n.advertiseService(BUT_GetUpdateTopic_SRV, getUpdateTopic);

  ROS_INFO("BUT GUI Service Server ready!");

  // Enters a loop, calling message callbacks
  ros::spin();

  return 0;
}

