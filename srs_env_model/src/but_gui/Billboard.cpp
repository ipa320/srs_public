/*
 *******************************************************************************
 * $Id: Billboard.cpp 146 2012-01-13 10:23:10Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 27.11.2011
 *******************************************************************************
 */

#include "but_gui/Billboard.h"

namespace but_gui
{

Billboard::Billboard(InteractiveMarkerServerPtr server_, string frame_id_, string name_)
{
  server = server_;
  frame_id = frame_id_;
  name = name_;
  velocity = 0.0;
}

Billboard::Billboard(InteractiveMarkerServerPtr server_, string frame_id_, string name_, int type_, Pose pose_,
                     Scale scale_)
{
  server = server_;
  frame_id = frame_id_;
  type = type_;
  name = name_;
  pose = pose_;
  scale = scale_;
  velocity = 0.0;
  create();
}

Billboard::~Billboard()
{
}

void Billboard::menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  menu_handler.getCheckState(handle, state);

  InteractiveMarker billboard;
  if (server->get(name, billboard))
  {
    server->erase(name);
    switch (feedback->menu_entry_id)
    {
      case 1:
        /**
         * Bounding box visibility
         */
        if (state == MenuHandler::CHECKED)
        {
          for (unsigned int i = 0; i < billboard.controls[1].markers.size(); i++)
          {
            billboard.controls[1].markers[i].color.a = 0.0;
          }
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          for (unsigned int i = 0; i < billboard.controls[1].markers.size(); i++)
          {
            billboard.controls[1].markers[i].color.a = 1.0;
          }
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 2:
        /**
         * Billboard description
         */
        if (state == MenuHandler::CHECKED)
        {
          billboard.controls[2].markers[0].color.a = 0.0;
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          billboard.controls[2].markers[0].color.a = 1.0;
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
    }
    server->insert(billboard);
  }

  menu_handler.reApply(*server);
  server->applyChanges();
}

void Billboard::setType(int type_)
{
  type = type_;
}

int Billboard::getType()
{
  return type;
}

void Billboard::setVelocity(double velocity_)
{
  velocity = velocity_;
}

double Billboard::getVelocity()
{
  return velocity;
}

void Billboard::setDirection(Quaternion direction_)
{
  direction = direction_;
}

Quaternion Billboard::getDirection()
{
  return direction;
}

void Billboard::createMenu()
{
  menu_handler.setCheckState(menu_handler.insert("Show movement", boost::bind(&Billboard::menuCallback, this, _1)),
                             MenuHandler::UNCHECKED);
  menu_handler.setCheckState(menu_handler.insert("Show description", boost::bind(&Billboard::menuCallback, this, _1)),
                             MenuHandler::UNCHECKED);
}

void Billboard::createMesh()
{
  mesh.type = Marker::MESH_RESOURCE;
  mesh.mesh_use_embedded_materials = true;
  mesh.scale.y = scale.x;
  mesh.scale.z = scale.y;

  if (type == BillboardType::CHAIR)
    mesh.mesh_resource = "package://srs_env_model/billboards/chair.mesh.xml";
  else if (type == BillboardType::MILK)
    mesh.mesh_resource = "package://srs_env_model/billboards/milk.mesh.xml";
  else if (type == BillboardType::TABLE)
    mesh.mesh_resource = "package://srs_env_model/billboards/table.mesh.xml";
  else if (type == BillboardType::PERSON)
    mesh.mesh_resource = "package://srs_env_model/billboards/person.mesh.xml";
  else
    ROS_ERROR("UNKNOWN BILLBOARD TYPE!");
}

void Billboard::createTrajectoryControl()
{

  trajectoryControl.name = "trajectory_control";
  trajectoryControl.always_visible = true;
  trajectoryControl.orientation_mode = InteractiveMarkerControl::FIXED;
  if (velocity != 0.0)
  {
    Marker trajectoryArrow;
    trajectoryArrow.type = Marker::ARROW;
    trajectoryArrow.pose.position.x = 0;
    trajectoryArrow.pose.position.y = 0;
    trajectoryArrow.pose.position.z = 0;
    trajectoryArrow.pose.orientation = direction;
    trajectoryArrow.scale.x = 0.25;
    trajectoryArrow.scale.y = 0.25;
    trajectoryArrow.scale.z = velocity;
    trajectoryArrow.color.r = 1.0;
    trajectoryArrow.color.g = 0.0;
    trajectoryArrow.color.b = 0.0;
    trajectoryArrow.color.a = 0.0;
    trajectoryControl.markers.push_back(trajectoryArrow);
  }

  ostringstream velocityText;
  velocityText << velocity << "m/s";
  Marker trajectoryText;
  trajectoryText.type = Marker::TEXT_VIEW_FACING;
  trajectoryText.text = velocityText.str();
  trajectoryText.scale.z = 0.2;
  trajectoryText.color.r = 1.0;
  trajectoryText.color.g = 0.0;
  trajectoryText.color.b = 0.0;
  trajectoryText.color.a = 0.0;
  trajectoryText.pose.position.x = 0.0;
  trajectoryText.pose.position.y = 0.0;
  trajectoryText.pose.position.z = 0.0;
  trajectoryControl.markers.push_back(trajectoryText);
}

void Billboard::create()
{
  object.header.frame_id = frame_id;
  object.header.stamp = ros::Time::now();
  object.name = name;
  object.description = name + " billboard";
  object.pose.position.x = pose.position.x;
  object.pose.position.y = pose.position.y;
  object.pose.position.z = pose.position.z;
  baseControlCount = 0;

  createMesh();

  control.name = "billboard_control";
  control.always_visible = true;
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.markers.push_back(mesh);
  object.controls.push_back(control);
  baseControlCount++;

  createTrajectoryControl();
  object.controls.push_back(trajectoryControl);
  baseControlCount++;

  createDescriptionControl();
  switch (type)
  {
    case BillboardType::PERSON:
      descriptionControl.markers[0].color.r = 1;
      descriptionControl.markers[0].color.g = 1;
      descriptionControl.markers[0].color.b = 0;
      break;
    case BillboardType::MILK:
      descriptionControl.markers[0].color.r = 0;
      descriptionControl.markers[0].color.g = 1;
      descriptionControl.markers[0].color.b = 1;
      break;
    case BillboardType::TABLE:
      descriptionControl.markers[0].color.r = 0;
      descriptionControl.markers[0].color.g = 1;
      descriptionControl.markers[0].color.b = 1;
      break;
    case BillboardType::CHAIR:
      descriptionControl.markers[0].color.r = 1;
      descriptionControl.markers[0].color.g = 0;
      descriptionControl.markers[0].color.b = 1;
      break;
  }

  object.controls.push_back(descriptionControl);
  baseControlCount++;

  createMenu();
}

void Billboard::insert()
{
  server->insert(object);
  menu_handler.apply(*server, name);
}

}

