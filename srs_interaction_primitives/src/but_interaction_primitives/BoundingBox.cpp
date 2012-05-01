/******************************************************************************
 * \file
 *
 * $Id: BoundingBox.cpp 676 2012-04-19 18:32:07Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 27/11/2011
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

#include <but_interaction_primitives/BoundingBox.h>

namespace but_interaction_primitives
{

BoundingBox::BoundingBox(InteractiveMarkerServerPtr server, string frame_id, string name) :
  Primitive(server, frame_id, name, srs_interaction_primitives::PrimitiveType::BOUNDING_BOX)
{
  description_ = "";
}

void BoundingBox::bboxCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->control_name != BOUNDING_BOX_CONTROL_NAME)
  {
    /*if ((feedback->event_type == InteractiveMarkerFeedback::MOUSE_UP))
     {
     InteractiveMarker o;
     if (server->get(name, o))
     {
     pose = o.pose;
     object.pose = pose;
     }

     ROS_INFO("New position:");
     cout << feedback->pose.position << endl;
     ROS_INFO("New orientation:");
     cout << feedback->pose.orientation << endl;

     // Interaction with object from Interactive Marker Server
     InteractiveMarker obj;
     if (server->get(attachedObjectName, obj))
     {
     server->erase(attachedObjectName);
     obj.pose = feedback->pose;
     server->insert(obj);
     }

     server->insert(object);
     menu_handler.reApply(*server);
     server->applyChanges();
     }*/
    defaultCallback(feedback);
  }
}

void BoundingBox::menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  string title;
  menu_handler_.getCheckState(handle, state);
  menu_handler_.getTitle(handle, title);

  updatePublisher_->publishMenuClicked(title, state);

  InteractiveMarker o;
  if (server_->get(name_, o))
  {
    pose_ = o.pose;
    object_.pose = pose_;
  }

  switch (feedback->menu_entry_id)
  {
    case 1:
      /*
       * Bounding box visibility
       */
      if (state == MenuHandler::CHECKED)
      {
        showBoundingBoxControl(false);
        menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
      }
      else
      {
        showBoundingBoxControl(true);
        menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
      }
      break;
    case 2:
      /*
       * Bounding box description
       */
      if (state == MenuHandler::CHECKED)
      {
        removeDescriptionControl();
        menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
      }
      else
      {
        addDescriptionControl();
        menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
      }
      break;
    case 3:
      /*
       * Bounding box measure
       */
      if (state == MenuHandler::CHECKED)
      {
        removeMeasureControl();
        menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
      }
      else
      {
        addMeasureControl();
        menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
      }
      break;
    case 5:
      /*
       * Movement controls
       */
      if (state == MenuHandler::CHECKED)
      {
        removeMovementControls();
        menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
      }
      else
      {
        addMovementControls();
        menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
      }
      break;
    case 6:
      /*
       * Rotation controls
       */
      if (state == MenuHandler::CHECKED)
      {
        removeRotationControls();
        menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
      }
      else
      {
        addRotationControls();
        menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
      }
      break;
    case 8:
      /*
       * Take object action
       */
      ROS_INFO("Take object");
      break;
    case 9:
      /*
       * Throw object action
       */
      ROS_INFO("Throw object");
      break;
  }

  server_->insert(object_);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

void BoundingBox::createMenu()
{
  if (!menu_created_)
  {
    menu_created_ = true;
    menu_handler_.setCheckState(menu_handler_.insert("Show Bounding Box", boost::bind(&BoundingBox::menuCallback, this,
                                                                                      _1)), MenuHandler::CHECKED);
    menu_handler_.setCheckState(menu_handler_.insert("Show description", boost::bind(&BoundingBox::menuCallback, this,
                                                                                     _1)), MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(
                                menu_handler_.insert("Show measure", boost::bind(&BoundingBox::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);

    MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Interaction");
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Movement",
                                                     boost::bind(&BoundingBox::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Rotation",
                                                     boost::bind(&BoundingBox::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);

    sub_menu_handle = menu_handler_.insert("Actions");
    menu_handler_.insert(sub_menu_handle, "Take object", boost::bind(&BoundingBox::menuCallback, this, _1));
    menu_handler_.insert(sub_menu_handle, "Throw object", boost::bind(&BoundingBox::menuCallback, this, _1));
  }
}

void BoundingBox::createBoundingBoxControl()
{
  Point p1, p2;
  double sx = scale_.x / 2;
  double sy = scale_.y / 2;
  double sz = scale_.z / 2;

  bounding_box_.type = Marker::CUBE;
  bounding_box_.scale = scale_;
  bounding_box_.color = color_;
  bounding_box_.color.a = BBOX_MAX_ALPHA;

  wire_.points.clear();

  wire_.type = Marker::LINE_LIST;
  wire_.scale = scale_;
  wire_.color.r = color_.b;
  wire_.color.g = color_.r;
  wire_.color.b = color_.g;
  wire_.color.a = BBOX_MAX_ALPHA;
  wire_.scale.x = 0.002;

  p1.x = -sx;
  p1.y = -sy;
  p1.z = -sz;
  p2.x = -sx;
  p2.y = sy;
  p2.z = -sz;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);
  p2.x = -sx;
  p2.y = -sy;
  p2.z = sz;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);
  p1.x = -sx;
  p1.y = sy;
  p1.z = sz;
  p2.x = -sx;
  p2.y = sy;
  p2.z = -sz;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);
  p2.x = -sx;
  p2.y = -sy;
  p2.z = sz;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);

  p1.x = sx;
  p1.y = -sy;
  p1.z = -sz;
  p2.x = sx;
  p2.y = sy;
  p2.z = -sz;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);
  p2.x = sx;
  p2.y = -sy;
  p2.z = sz;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);
  p1.x = sx;
  p1.y = sy;
  p1.z = sz;
  p2.x = sx;
  p2.y = sy;
  p2.z = -sz;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);
  p2.x = sx;
  p2.y = -sy;
  p2.z = sz;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);

  p1.x = sx;
  p1.y = sy;
  p1.z = sz;
  p2.x = -sx;
  p2.y = sy;
  p2.z = sz;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);

  p1.x = sx;
  p1.y = -sy;
  p1.z = sz;
  p2.x = -sx;
  p2.y = -sy;
  p2.z = sz;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);

  p1.x = sx;
  p1.y = sy;
  p1.z = -sz;
  p2.x = -sx;
  p2.y = sy;
  p2.z = -sz;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);

  p1.x = sx;
  p1.y = -sy;
  p1.z = -sz;
  p2.x = -sx;
  p2.y = -sy;
  p2.z = -sz;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);

  control_.name = BOUNDING_BOX_CONTROL_NAME;
  control_.markers.push_back(bounding_box_);
  control_.markers.push_back(wire_);
  control_.interaction_mode = InteractiveMarkerControl::BUTTON;
  control_.always_visible = true;

  object_.controls.push_back(control_);
}

void BoundingBox::showBoundingBoxControl(bool show)
{
  for (unsigned int i = 0; i < control_.markers.size(); i++)
    control_.markers[i].color.a = show ? BBOX_MAX_ALPHA : BBOX_MIN_ALPHA;

  removeControl(BOUNDING_BOX_CONTROL_NAME);
  object_.controls.push_back(control_);
}

void BoundingBox::create()
{
  clearObject();

  object_.header.frame_id = frame_id_;
  object_.header.stamp = ros::Time::now();
  object_.name = name_;
  object_.description = description_;
  object_.pose = pose_;
  object_.scale = but_interaction_primitives::maxScale(scale_);

  createBoundingBoxControl();

  createMenu();
}

void BoundingBox::insert()
{
  create();

  server_->insert(object_, boost::bind(&BoundingBox::bboxCallback, this, _1));
  menu_handler_.apply(*server_, name_);
}

}
