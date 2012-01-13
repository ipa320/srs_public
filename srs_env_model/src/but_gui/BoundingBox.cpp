/*
 *******************************************************************************
 * $Id: BoundingBox.cpp 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 27.11.2011
 *******************************************************************************
 */

#include "but_gui/BoundingBox.h"

namespace but_gui
{

BoundingBox::BoundingBox(InteractiveMarkerServerPtr server_, string frame_id_, string name_)
{
  server = server_;
  frame_id = frame_id_;
  name = name_;
  immediateInteraction = false;
  description = "";
}

BoundingBox::BoundingBox(InteractiveMarkerServerPtr server_, string frame_id_, string name_, string objectName_,
                         Pose pose_, Scale scale_, ColorRGBA color_, string description_)
{
  server = server_;
  frame_id = frame_id_;
  attachedObjectName = objectName_;
  name = name_;
  pose = pose_;
  scale = scale_;
  color = color_;
  immediateInteraction = false;
  description = description_;
  create();
}

BoundingBox::~BoundingBox()
{
}

string BoundingBox::getAttachedObjectName()
{
  return attachedObjectName;
}

void BoundingBox::setAttachedObjectName(string name_)
{
  attachedObjectName = name_;
}

void BoundingBox::bboxCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->control_name != "box_control")
  {
    if (immediateInteraction || (feedback->event_type == InteractiveMarkerFeedback::MOUSE_UP))
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
    }
  }
}

void BoundingBox::menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  menu_handler.getCheckState(handle, state);

  InteractiveMarker bounding_box;
  if (server->get(name, bounding_box))
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
          bounding_box.controls[0].markers[0].color.a = BBOX_MIN_ALPHA;
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          bounding_box.controls[0].markers[0].color.a = BBOX_MAX_ALPHA;
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 2:
        /**
         * Bounding box description
         */
        if (state == MenuHandler::CHECKED)
        {
          bounding_box.controls[1].markers[0].color.a = 0.0;
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          bounding_box.controls[1].markers[0].color.a = 1.0;
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 3:
        /**
         * Bounding box measure
         */
        if (state == MenuHandler::CHECKED)
        {
          for (unsigned int i = 0; i < bounding_box.controls[2].markers.size(); i++)
          {
            bounding_box.controls[2].markers[i].color.a = 0.0;
          }
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          for (unsigned int i = 0; i < bounding_box.controls[2].markers.size(); i++)
          {
            bounding_box.controls[2].markers[i].color.a = 1.0;
          }
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 5:
        /*
         * Movement controls
         */
        if (state == MenuHandler::CHECKED)
        {
          MenuHandler::EntryHandle h = 6;
          MenuHandler::CheckState s;
          menu_handler.getCheckState(h, s);
          removeMovementControls(bounding_box, s == MenuHandler::CHECKED);
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          addMovementControls(bounding_box);
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 6:
        /*
         * Rotation controls
         */
        if (state == MenuHandler::CHECKED)
        {
          MenuHandler::EntryHandle h = 5;
          MenuHandler::CheckState s;
          menu_handler.getCheckState(h, s);
          removeRotationControls(bounding_box, s == MenuHandler::CHECKED);
          menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          addRotationControls(bounding_box);
          menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 7:
        /*
         * Immediate interaction
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
    server->insert(bounding_box);
  }

  menu_handler.reApply(*server);
  server->applyChanges();
}

void BoundingBox::createMenu()
{
  menu_handler.setCheckState(
                             menu_handler.insert("Show Bounding Box", boost::bind(&BoundingBox::menuCallback, this, _1)),
                             MenuHandler::CHECKED);
  menu_handler.setCheckState(
                             menu_handler.insert("Show description", boost::bind(&BoundingBox::menuCallback, this, _1)),
                             MenuHandler::UNCHECKED);
  menu_handler.setCheckState(menu_handler.insert("Show measure", boost::bind(&BoundingBox::menuCallback, this, _1)),
                             MenuHandler::UNCHECKED);

  MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert("Interaction");
  menu_handler.setCheckState(menu_handler.insert(sub_menu_handle, "Movement", boost::bind(&BoundingBox::menuCallback,
                                                                                          this, _1)),
                             MenuHandler::UNCHECKED);
  menu_handler.setCheckState(menu_handler.insert(sub_menu_handle, "Rotation", boost::bind(&BoundingBox::menuCallback,
                                                                                          this, _1)),
                             MenuHandler::UNCHECKED);
  menu_handler.setCheckState(menu_handler.insert(sub_menu_handle, "Immediate", boost::bind(&BoundingBox::menuCallback,
                                                                                           this, _1)),
                             MenuHandler::UNCHECKED);

  sub_menu_handle = menu_handler.insert("Actions");
  menu_handler.insert(sub_menu_handle, "Take object", boost::bind(&BoundingBox::menuCallback, this, _1));
  menu_handler.insert(sub_menu_handle, "Throw object", boost::bind(&BoundingBox::menuCallback, this, _1));
}

void BoundingBox::createBoundingBox()
{
  object.header.frame_id = frame_id;
  object.header.stamp = ros::Time::now();
  object.name = name;
  object.description = description;
  object.pose = pose;
  object.scale = but_gui::maxScale(scale);
}

void BoundingBox::createBox()
{
  box.type = Marker::CUBE;
  box.scale = scale;
  box.color = color;
  box.color.a = BBOX_MAX_ALPHA;
}

void BoundingBox::create()
{
  baseControlCount = 0;

  createControls();
  createBoundingBox();

  control.name = "box_control";
  createBox();
  control.markers.push_back(box);
  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  object.controls.push_back(control);
  baseControlCount++;

  createDescriptionControl();
  object.controls.push_back(descriptionControl);
  baseControlCount++;

  createMeasureControl();
  object.controls.push_back(measureControl);
  baseControlCount++;

  createMenu();
}

void BoundingBox::insert()
{
  server->insert(object, boost::bind(&BoundingBox::bboxCallback, this, _1));
  menu_handler.apply(*server, name);
}

}
