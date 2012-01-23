/*
 *******************************************************************************
 * $Id: UnknownObject.cpp 146 2012-01-13 10:23:10Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 11.12.2011
 * Description:
 *******************************************************************************
 */

#include "but_gui/UnknownObject.h"

namespace but_gui
{

UnknownObject::UnknownObject(InteractiveMarkerServerPtr server_, string frame_id_, string name_)
{
  server = server_;
  frame_id = frame_id_;
  name = name_;
  immediateInteraction = false;
  description = "";
  color.r = 0.3;
  color.g = 0.5;
  color.b = 0.6;
  color.a = 1.0;
}

UnknownObject::UnknownObject(InteractiveMarkerServerPtr server_, string frame_id_, string name_, Pose pose_,
                             Scale scale_, string description_)
{
  server = server_;
  frame_id = frame_id_;
  name = name_;
  pose = pose_;
  scale = scale_;
  immediateInteraction = false;
  description = description_;
  color.r = 0.3;
  color.g = 0.5;
  color.b = 0.6;
  color.a = 1.0;
  create();
}

UnknownObject::~UnknownObject()
{
}

void UnknownObject::uboxCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->control_name == "measure_control")
  {
    if (feedback->event_type == InteractiveMarkerFeedback::MOUSE_UP)
    {
      InteractiveMarker unknown_object;
      if (server->get(name, unknown_object))
      {
        ROS_INFO("SCALING");

        cout << feedback->pose.position << endl;
        unknown_object.controls[0].markers[0].scale.x += feedback->pose.orientation.x;

        server->erase(name);
        scale.x += feedback->pose.orientation.x;
        create();
        server->insert(object);
        server->applyChanges();
      }
    }
    /*
     if (feedback->event_type == InteractiveMarkerFeedback::MOUSE_UP)
     {
     ROS_INFO("New position:");
     cout << feedback->pose.position << endl;
     ROS_INFO("New orientation:");
     cout << feedback->pose.orientation << endl;

     // Interaction with object from Interactive Marker Server
     // TODO what to do with other objects?
     InteractiveMarker object;
     if (server->get(attachedObjectName, object))
     {
     server->erase(attachedObjectName);
     object.pose = feedback->pose;
     server->insert(object);
     server->applyChanges();
     }
     }*/
  }
}

void UnknownObject::menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  menu_handler.getCheckState(handle, state);

  InteractiveMarker unknown_object;
  if (server->get(name, unknown_object))
  {
    server->erase(name);
    switch (feedback->menu_entry_id)
    {
      case 1:
        /**
         * Bounding box description
         */
        if (state == MenuHandler::CHECKED)
        {
          unknown_object.controls[1].markers[0].color.a = 0.0;
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          unknown_object.controls[1].markers[0].color.a = 1.0;
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 2:
        /**
         * Bounding box measure
         */
        if (state == MenuHandler::CHECKED)
        {
          for (unsigned int i = 0; i < unknown_object.controls[2].markers.size(); i++)
          {
            unknown_object.controls[2].markers[i].color.a = 0.0;
          }
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          for (unsigned int i = 0; i < unknown_object.controls[2].markers.size(); i++)
          {
            unknown_object.controls[2].markers[i].color.a = 1.0;
          }
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 4:
        /*
         * Movement controls
         */
        if (state == MenuHandler::CHECKED)
        {
          MenuHandler::EntryHandle h = 5;
          MenuHandler::CheckState s;
          menu_handler.getCheckState(h, s);
          removeMovementControls(unknown_object, s == MenuHandler::CHECKED);
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          addMovementControls(unknown_object);
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 5:
        /*
         * Rotation controls
         */
        if (state == MenuHandler::CHECKED)
        {
          MenuHandler::EntryHandle h = 4;
          MenuHandler::CheckState s;
          menu_handler.getCheckState(h, s);
          removeRotationControls(unknown_object, s == MenuHandler::CHECKED);
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          addRotationControls(unknown_object);
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 7:
        /*
         * Scale controls
         * TODO add this controls
         */
        if (state == MenuHandler::CHECKED)
        {
          immediateInteraction = false;
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          immediateInteraction = true;
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 9:
        /*
         * Take object action
         */
        ROS_INFO("Take object");
        break;
      case 10:
        /*
         * Throw object action
         */
        ROS_INFO("Throw object");
        break;
    }
    server->insert(unknown_object);
  }

  menu_handler.reApply(*server);
  server->applyChanges();
}

void UnknownObject::createMenu()
{
  menu_handler.setCheckState(menu_handler.insert("Show description",
                                                 boost::bind(&UnknownObject::menuCallback, this, _1)),
                             MenuHandler::UNCHECKED);
  menu_handler.setCheckState(menu_handler.insert("Show measure", boost::bind(&UnknownObject::menuCallback, this, _1)),
                             MenuHandler::UNCHECKED);

  MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert("Interaction");
  menu_handler.setCheckState(menu_handler.insert(sub_menu_handle, "Movement", boost::bind(&UnknownObject::menuCallback,
                                                                                          this, _1)),
                             MenuHandler::UNCHECKED);
  menu_handler.setCheckState(menu_handler.insert(sub_menu_handle, "Rotation", boost::bind(&UnknownObject::menuCallback,
                                                                                          this, _1)),
                             MenuHandler::UNCHECKED);
  menu_handler.setCheckState(menu_handler.insert(sub_menu_handle, "Scale", boost::bind(&UnknownObject::menuCallback,
                                                                                       this, _1)),
                             MenuHandler::UNCHECKED);
}

void UnknownObject::createBox()
{
  box.type = Marker::MESH_RESOURCE;
  box.mesh_use_embedded_materials = true;
  box.scale = scale;
  box.mesh_resource = "package://srs_env_model/meshes/unknown_object.mesh.xml";
}

void UnknownObject::createUnknownBox()
{
  object.header.frame_id = frame_id;
  object.header.stamp = ros::Time::now();
  object.name = name;
  object.description = description;
  object.pose = pose;
  object.scale = but_gui::maxScale(scale);
}

void UnknownObject::create()
{
  baseControlCount = 0;

  createControls();
  createUnknownBox();

  control.name = "box_control";
  createBox();
  control.markers.push_back(box);
  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.always_visible = true;
  object.controls.push_back(control);
  baseControlCount++;

  createDescriptionControl();
  object.controls.push_back(descriptionControl);
  baseControlCount++;

  createMeasureControl();
  measureControl.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  object.controls.push_back(measureControl);
  baseControlCount++;

  /*InteractiveMarkerControl cScaleX;
   cScaleX.name = "scale_x";
   cScaleX.set_markers_size(0.5);
   cScaleX.orientation.w = 1;
   cScaleX.orientation.x = 1;
   cScaleX.orientation.y = 0;
   cScaleX.orientation.z = 0;
   cScaleX.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
   object.controls.push_back(cScaleX);*/

  createMenu();
}

void UnknownObject::insert()
{
  //  server->insert(object);
  server->insert(object, boost::bind(&UnknownObject::uboxCallback, this, _1));//
  menu_handler.apply(*server, name);
}

}
