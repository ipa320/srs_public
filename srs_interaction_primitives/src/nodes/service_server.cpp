/******************************************************************************
 * \file
 *
 * $Id: interaction_primitives_service_server.cpp 676 2012-04-19 18:32:07Z xlokaj03 $
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

#include <srs_interaction_primitives/service_server.h>

using namespace std;
using namespace interactive_markers;
using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std_msgs;

namespace srs_interaction_primitives
{

// Container with primitives
std::map<std::string, Primitive*> primitives;

// Interactive Marker server
InteractiveMarkerServerPtr imServer;

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
  plane->setPoseType(req.pose_type);
  plane->setPose(req.pose);
  plane->setScale(req.scale);
  plane->setColor(req.color);
  plane->insert();
  imServer->applyChanges();

  primitives.insert(make_pair(req.name, plane));

  ROS_INFO("..... DONE");
  return true;
}

bool addPlanePolygon(AddPlanePolygon::Request &req, AddPlanePolygon::Response &res)
{
  ROS_INFO("ADDING PLANE POLYGON");

  InteractiveMarker tmp;
  if ((imServer->get(req.name, tmp)) || (primitives.count(req.name) != 0))
  {
    ROS_ERROR("Object with that name already exists! Please remove it first.");
    return false;
  }

  PlanePolygon *plane = new PlanePolygon(imServer, req.frame_id, req.name);
  plane->setPolygon(req.polygon);
  plane->setNormal(req.normal);
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
  billboard->setPoseType(req.pose_type);
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
  boundingBox->setPoseType(req.pose_type);
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
  if (req.resource == "")
  {
    object->setShape(req.shape);
  }
  else
  {
    object->setResource(req.resource);
    object->setUseMaterial(req.use_material);
  }
  //  object->setPose(req.pose);
  object->setPoseType(req.pose_type);
  object->setPoseLWH(req.pose, req.bounding_box_lwh);
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
  unknownObject->setPoseType(req.pose_type);
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

bool setPreGraspPosition(SetPreGraspPosition::Request &req, SetPreGraspPosition::Response &res)
{
  ROS_INFO("SETTING PRE-GRASP POSITION");

  InteractiveMarker tmp;
  if ((!imServer->get(req.name, tmp)) || (primitives.count(req.name) != 1))
  {
    ROS_ERROR("Primitive with that name doesn't exist!");
    return false;
  }

  primitives[req.name]->addPreGraspPosition(req.pos_id, req.pose);
  primitives[req.name]->insert();
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

bool removePreGraspPosition(RemovePreGraspPosition::Request &req, RemovePreGraspPosition::Response &res)
{
  ROS_INFO("REMOVING PRE-GRASP POSITION");

  InteractiveMarker tmp;
  if ((!imServer->get(req.name, tmp)) || (primitives.count(req.name) != 1))
  {
    ROS_ERROR("Primitive with that name doesn't exist!");
    return false;
  }

  primitives[req.name]->removePreGraspPosition(req.pos_id);
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
    ROS_ERROR("Primitive with that name doesn't exist!");
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
    ROS_ERROR("Primitive with that name doesn't exist!");
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
    ROS_ERROR("Primitive with that name doesn't exist!");
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
    ROS_ERROR("Primitive with that name doesn't exist!");
    return false;
  }

  res.old_color = primitives[req.name]->getColor();

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
    ROS_ERROR("Primitive with that name doesn't exist!");
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
    ROS_ERROR("Primitive with that name doesn't exist!");
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

bool setAllowObjectInteraction(SetAllowObjectInteraction::Request &req, SetAllowObjectInteraction::Response &res)
{
  ROS_INFO("SETTING OBJECT'S INTERACTION");

  InteractiveMarker tmp;
  if ((!imServer->get(req.name, tmp)) || (primitives.count(req.name) != 1))
  {
    ROS_ERROR("Primitive with that name doesn't exist!");
    return false;
  }

  primitives[req.name]->setAllowObjectInteraction(req.allow);

  ROS_INFO("..... DONE");
  return true;
}

bool getUpdateTopic(GetUpdateTopic::Request &req, GetUpdateTopic::Response &res)
{
  ROS_INFO("GETTING UPDATE TOPIC");

  InteractiveMarker tmp;
  if ((!imServer->get(req.name, tmp)) || (primitives.count(req.name) != 1))
  {
    ROS_ERROR("Primitive with that name doesn't exist!");
    return false;
  }

  res.update_topic = primitives[req.name]->getUpdateTopic(req.type);

  ROS_INFO("..... DONE");
  return true;
}

bool getAllPrimitivesNames(GetAllPrimitivesNames::Request &req, GetAllPrimitivesNames::Response &res)
{
  ROS_INFO("GETTING ALL PRIMITIVE'S NAMES");

  map<string, Primitive*>::iterator i;
  for (i = primitives.begin(); i != primitives.end(); i++)
    res.primitives_names.push_back(i->second->getName());

  return true;
}

bool getObject(GetObject::Request &req, GetObject::Response &res)
{
  ROS_INFO("GETTING OBJECT");

  if (primitives.count(req.name) > 0)
  {
    Object *obj = (Object*)primitives.at(req.name);
    if (obj->getPrimitiveType() == srs_interaction_primitives::PrimitiveType::OBJECT)
    {
      res.name = req.name;
      res.frame_id = obj->getFrameID();
      res.pose_type = obj->getPoseType();
      res.pose = obj->getPose();
      res.bounding_box_lwh = obj->getBoundingBoxLWH();
      res.description = obj->getDescription();
      res.color = obj->getColor();
      res.shape = obj->getShape();
      res.resource = obj->getResource();
      res.use_material = obj->getUseMaterial();
      return true;
    }
  }
  ROS_ERROR("Object with that name doesn't exist!");
  return false;
}

bool getUnknownObject(GetUnknownObject::Request &req, GetUnknownObject::Response &res)
{
  ROS_INFO("GETTING OBJECT");

  if (primitives.count(req.name) > 0)
  {
    UnknownObject *obj = (UnknownObject*)primitives.at(req.name);
    if (obj->getPrimitiveType() == srs_interaction_primitives::PrimitiveType::UNKNOWN_OBJECT)
    {
      res.name = req.name;
      res.frame_id = obj->getFrameID();
      res.pose_type = obj->getPoseType();
      res.pose = obj->getPose();
      res.description = obj->getDescription();
      res.scale = obj->getScale();
      return true;
    }
  }
  ROS_ERROR("Unknown Object with that name doesn't exist!");
  return false;
}

bool getBillboard(GetBillboard::Request &req, GetBillboard::Response &res)
{
  ROS_INFO("GETTING BILLBOARD");

  if (primitives.count(req.name) > 0)
  {
    Billboard *obj = (Billboard*)primitives.at(req.name);
    if (obj->getPrimitiveType() == srs_interaction_primitives::PrimitiveType::BILLBOARD)
    {
      res.name = req.name;
      res.frame_id = obj->getFrameID();
      res.pose_type = obj->getPoseType();
      res.pose = obj->getPose();
      res.description = obj->getDescription();
      res.scale = obj->getScale();
      res.description = obj->getDescription();
      res.type = obj->getType();
      res.velocity = obj->getVelocity();
      res.direction = obj->getDirection();
      return true;
    }
  }
  ROS_ERROR("Billboard with that name doesn't exist!");
  return false;
}

bool getBoundingBox(GetBoundingBox::Request &req, GetBoundingBox::Response &res)
{
  ROS_INFO("GETTING BOUNDING BOX");

  if (primitives.count(req.name) > 0)
  {
    BoundingBox *obj = (BoundingBox*)primitives.at(req.name);
    if (obj->getPrimitiveType() == srs_interaction_primitives::PrimitiveType::BOUNDING_BOX)
    {
      res.name = req.name;
      res.frame_id = obj->getFrameID();
      res.pose_type = obj->getPoseType();
      res.pose = obj->getPose();
      res.description = obj->getDescription();
      res.color = obj->getColor();
      res.scale = obj->getScale();
      res.object_name = obj->getAttachedObjectName();
      return true;
    }
    ROS_ERROR("Bounding box with that name doesn't exist!");
    return false;
  }
}

bool getPlane(GetPlane::Request &req, GetPlane::Response &res)
{
  ROS_INFO("GETTING PLANE");

  if (primitives.count(req.name) > 0)
  {
    Plane *obj = (Plane*)primitives.at(req.name);
    if (obj->getPrimitiveType() == srs_interaction_primitives::PrimitiveType::PLANE)
    {
      res.name = req.name;
      res.frame_id = obj->getFrameID();
      res.pose_type = obj->getPoseType();
      res.pose = obj->getPose();
      res.description = obj->getDescription();
      res.color = obj->getColor();
      res.scale = obj->getScale();
      return true;
    }
    ROS_ERROR("Plane with that name doesn't exist!");
    return false;
  }
}
}

/**
 * @brief Main function
 */
int main(int argc, char **argv)
{
  using namespace srs_interaction_primitives;

  // ROS initialization (the last argument is the name of the node)
  ros::init(argc, argv, "interaction_primitives_service_server");

  // Interactive Marker Server initialization
  imServer.reset(new InteractiveMarkerServer("but_interaction_primitives", "", false));
  imServer->clear();

  // NodeHandle is the main access point to communications with the ROS system
  ros::NodeHandle n;

  // Create and advertise this service over ROS
  ros::ServiceServer addBoundingBoxService = n.advertiseService(AddBoundingBox_SRV, addBoundingBox);
  ros::ServiceServer addBillboardService = n.advertiseService(AddBillboard_SRV, addBillboard);
  ros::ServiceServer addPlaneService = n.advertiseService(AddPlane_SRV, addPlane);
  ros::ServiceServer addPlanePolygonService = n.advertiseService(AddPlanePolygon_SRV, addPlanePolygon);
  ros::ServiceServer addObjectService = n.advertiseService(AddObject_SRV, addObject);
  ros::ServiceServer addUnknownObjectService = n.advertiseService(AddUnknownObject_SRV, addUnknownObject);

  ros::ServiceServer removePrimitiveService = n.advertiseService(RemovePrimitive_SRV, removePrimitive);

  ros::ServiceServer changeDescriptionService = n.advertiseService(ChangeDescription_SRV, changeDescription);
  ros::ServiceServer changePoseService = n.advertiseService(ChangePose_SRV, changePose);
  ros::ServiceServer changeScaleService = n.advertiseService(ChangeScale_SRV, changeScale);
  ros::ServiceServer changeColorService = n.advertiseService(ChangeColor_SRV, changeColor);
  ros::ServiceServer changeDirectionService = n.advertiseService(ChangeDirection_SRV, changeDirection);
  ros::ServiceServer changeVelocityService = n.advertiseService(ChangeVelocity_SRV, changeVelocity);
  ros::ServiceServer setAllowObjectInteractionService = n.advertiseService(SetAllowObjectInteraction_SRV,
                                                                           setAllowObjectInteraction);

  ros::ServiceServer setGraspingPositionService = n.advertiseService(SetGraspingPosition_SRV, setPreGraspPosition);
  ros::ServiceServer removeGraspingPositionService = n.advertiseService(RemoveGraspingPosition_SRV,
                                                                        removePreGraspPosition);

  ros::ServiceServer getUpdateTopicService = n.advertiseService(GetUpdateTopic_SRV, getUpdateTopic);

  ros::ServiceServer getAllPrimitivesNamesService = n.advertiseService(GetAllPrimitivesNames_SRV,
                                                                       getAllPrimitivesNames);
  ros::ServiceServer getObjectService = n.advertiseService(GetObject_SRV, getObject);
  ros::ServiceServer getUnknownObjectService = n.advertiseService(GetUnknownObject_SRV, getUnknownObject);
  ros::ServiceServer getBillboardService = n.advertiseService(GetBillboard_SRV, getBillboard);
  ros::ServiceServer getBoundingBoxService = n.advertiseService(GetBoundingBox_SRV, getBoundingBox);
  ros::ServiceServer getPlaneService = n.advertiseService(GetPlane_SRV, getPlane);

  ROS_INFO("Interaction Primitives Service Server ready!");

  // Enters a loop, calling message callbacks
  ros::spin();

  return 0;
}

