/*
 ********************************************************************************
 * $Id: but_gui_services.cpp 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 9.12.2011
 *
 * Description:
 * This server advertises services for but_gui
 *******************************************************************************
 */

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <srs_env_model/AddObject.h>
#include <srs_env_model/AddMarker.h>
#include <srs_env_model/AddBoundingBox.h>
#include <srs_env_model/AddBillboard.h>
#include <srs_env_model/AddPlane.h>
#include <srs_env_model/RemoveObject.h>
#include <but_gui/GuiObject.h>
#include <but_gui/BoundingBox.h>
#include <but_gui/Billboard.h>
#include <but_gui/Plane.h>
#include <math.h>

using namespace but_gui;

//------------------------------------------------------------------------------
// Interactive Marker server
but_gui::InteractiveMarkerServerPtr imServer;

bool addPlane(AddPlane::Request &req, AddPlane::Response &res)
{
  /* AddPlane adding.
   *
   * @param req  Request of type AddPlane.
   * @param res  Response of type AddPlane.
   */
  ROS_INFO("ADDING PLANE");

  InteractiveMarker tmp;
  if (imServer->get(req.name, tmp))
  {
    ROS_ERROR("Object with that name already exists! Please remove it first.");
    return false;
  }

  Plane *plane = new Plane(imServer, req.frame_id, req.name, req.pose, req.scale, req.color);
  plane->insert();
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

bool addBillboard(AddBillboard::Request &req, AddBillboard::Response &res)
{
  /* AddBillboard adding.
   *
   * @param req  Request of type AddBillboard.
   * @param res  Response of type AddBillboard.
   */
  ROS_INFO("ADDING BILLBOARD");

  InteractiveMarker tmp;
  if (imServer->get(req.name, tmp))
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
  billboard->create();
  billboard->insert();
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

bool addBoundingBox(AddBoundingBox::Request &req, AddBoundingBox::Response &res)
{
  /* Bounding Box adding.
   *
   * @param req  Request of type AddBoundingBox.
   * @param res  Response of type AddBoundingBox.
   */
  ROS_INFO("ADDING BOUNDING BOX");

  InteractiveMarker tmp;
  if (imServer->get(req.name, tmp))
  {
    ROS_ERROR("Object with that name already exists! Please remove it first.");
    return false;
  }

  BoundingBox * boundingBox = new BoundingBox(imServer, req.frame_id, req.name, req.object_name, req.pose, req.scale,
                                              req.color, req.description);
  boundingBox->insert();
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

bool addMarker(AddMarker::Request &req, AddMarker::Response &res)
{
  /* Marker adding.
   *
   * @param req  Request of type AddMarker.
   * @param res  Response of type AddMarker.
   */
  ROS_INFO("ADDING MARKER");

  InteractiveMarker tmp;
  if (imServer->get(req.name, tmp))
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

bool addObject(AddObject::Request &req, AddObject::Response &res)
{
  return true;
}

bool removeObject(RemoveObject::Request &req, RemoveObject::Response &res)
{
  /* Object removing.
   *
   * @param req  Request of type RemoveObject.
   * @param res  Response of type RemoveObject.
   */
  ROS_INFO("REMOVING OBJECT");

  InteractiveMarker tmp;
  if (!imServer->get(req.name, tmp))
  {
    ROS_ERROR("Object with that name doesn't exist!");
    return false;
  }

  imServer->erase(req.name);
  imServer->applyChanges();

  ROS_INFO("..... DONE");
  return true;
}

int main(int argc, char **argv)
{
  /*
   * Main function
   */

  // ROS initialization (the last argument is the name of the node)
  ros::init(argc, argv, "but_gui_server");

  // Interactive Marker Server initialization
  imServer.reset(new InteractiveMarkerServer("but_gui", "", false));

  // NodeHandle is the main access point to communications with the ROS system
  ros::NodeHandle n;

  // Create and advertise this service over ROS
  //ros::ServiceServer addObjectService = n.advertiseService("add_object", addObject);
  ros::ServiceServer addMarkerService = n.advertiseService("add_marker", addMarker);
  ros::ServiceServer addBoundingBoxService = n.advertiseService("add_bounding_box", addBoundingBox);
  ros::ServiceServer addBillboardService = n.advertiseService("add_billboard", addBillboard);
  ros::ServiceServer addPlaneService = n.advertiseService("add_plane", addPlane);

  ros::ServiceServer removeObjectService = n.advertiseService("remove_object", removeObject);


  ROS_INFO("BUT_GUI Service Server ready!");

  // Enters a loop, calling message callbacks
  ros::spin();

  return 0;
}

