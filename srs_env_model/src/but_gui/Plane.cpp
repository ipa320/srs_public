/*
 *******************************************************************************
 * $Id: Plane.cpp 146 2012-01-13 10:23:10Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 2.12.2011
 *******************************************************************************
 */

#include "but_gui/Plane.h"

namespace but_gui
{

Plane::Plane(InteractiveMarkerServerPtr server_, string frame_id_, string name_)
{
  server = server_;
  frame_id = frame_id_;
  name = name_;
}

Plane::Plane(InteractiveMarkerServerPtr server_, string frame_id_, string name_, Pose pose_, Scale scale_,
             ColorRGBA color_)
{
  server = server_;
  frame_id = frame_id_;
  name = name_;
  pose = pose_;
  scale = scale_;
  color = color_;
  create();
}

Plane::~Plane()
{
}

void Plane::menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  menu_handler.getCheckState(handle, state);

  InteractiveMarker plane;
  if (server->get(name, plane))
  {
    server->erase(name);
    switch (feedback->menu_entry_id)
    {
      case 1:
        /**
         * Plane tag description
         */
        if (state == MenuHandler::CHECKED)
        {
          plane.controls[1].markers[0].color.a = 0.0;
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          plane.controls[1].markers[0].color.a = 1.0;
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 3:
        /**
         * Plane tag
         */
        tag = "Table desk";
        if (state == MenuHandler::CHECKED)
        {
          plane.controls[1].markers[0].text = tag;
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          plane.controls[1].markers[0].text = tag;
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 4:
        /**
         * Plane tag
         */
        tag = "Wall";
        if (state == MenuHandler::CHECKED)
        {
          plane.controls[1].markers[0].text = tag;
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          plane.controls[1].markers[0].text = tag;
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 5:
        /**
         * Plane tag
         */
        tag = "Door";
        if (state == MenuHandler::CHECKED)
        {
          plane.controls[1].markers[0].text = tag;
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          plane.controls[1].markers[0].text = tag;
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
    }

    server->insert(plane);
  }

  menu_handler.reApply(*server);
  server->applyChanges();
}

void Plane::createMenu()
{
  menu_handler.setCheckState(menu_handler.insert("Show description", boost::bind(&Plane::menuCallback, this, _1)),
                             MenuHandler::UNCHECKED);
  MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert("Tag");
  menu_handler.setCheckState(menu_handler.insert(sub_menu_handle, "Table desk", boost::bind(&Plane::menuCallback, this,
                                                                                            _1)),
                             MenuHandler::UNCHECKED);
  menu_handler.setCheckState(menu_handler.insert(sub_menu_handle, "Wall", boost::bind(&Plane::menuCallback, this, _1)),
                             MenuHandler::UNCHECKED);
  menu_handler.setCheckState(menu_handler.insert(sub_menu_handle, "Door", boost::bind(&Plane::menuCallback, this, _1)),
                             MenuHandler::UNCHECKED);
}

void Plane::create()
{
  object.header.frame_id = frame_id;
  object.header.stamp = ros::Time::now();
  object.name = name;
  object.description = name + " plane";
  object.pose = pose;

  mesh.type = Marker::MESH_RESOURCE;
  mesh.color = color;
  // Todo switch x and y ?
  mesh.scale = scale;
  mesh.mesh_resource = "package://srs_env_model/meshes/plane.mesh.xml";

  control.always_visible = true;
  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.markers.push_back(mesh);
  object.controls.push_back(control);
  baseControlCount++;

  createDescriptionControl();
  descriptionControl.markers[0].text = tag;
  object.controls.push_back(descriptionControl);
  baseControlCount++;

  createMenu();
}

void Plane::insert()
{
  server->insert(object);
  menu_handler.apply(*server, name);
}

}
