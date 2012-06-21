/******************************************************************************
 * \file
 *
 * $Id: Plane.cpp 676 2012-04-19 18:32:07Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 2/12/2011
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

#include <srs_interaction_primitives/plane.h>

using namespace std;
using namespace interactive_markers;
using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std_msgs;


namespace srs_interaction_primitives
{

Plane::Plane(InteractiveMarkerServerPtr server, string frame_id, string name) :
  Primitive(server, frame_id, name, srs_interaction_primitives::PrimitiveType::PLANE)
{
}

void Plane::menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  string title;
  menu_handler_.getCheckState(handle, state);
  menu_handler_.getTitle(handle, title);

  updatePublisher_->publishMenuClicked(title, state);

  switch (feedback->menu_entry_id)
  {
    case 1:
      /*
       * Plane tag description
       */
      if (state == MenuHandler::CHECKED)
      {
        removeDescriptionControl();
        menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
      }
      else
      {
        description_ = tag_;
        addDescriptionControl();
        menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
      }
      break;
    case 3:
      /*
       * Plane tag
       */
      tag_ = "Unknown";
      updatePublisher_->publishTagChanged(tag_);
      if (state == MenuHandler::UNCHECKED)
      {
        menu_handler_.setCheckState(3, MenuHandler::CHECKED);
        menu_handler_.setCheckState(4, MenuHandler::UNCHECKED);
        menu_handler_.setCheckState(5, MenuHandler::UNCHECKED);
        menu_handler_.setCheckState(6, MenuHandler::UNCHECKED);
        if (show_description_control_)
        {
          removeDescriptionControl();
          description_ = tag_;
          addDescriptionControl();
        }
      }
      break;
    case 4:
      /*
       * Plane tag
       */
      tag_ = "Wall";
      updatePublisher_->publishTagChanged(tag_);
      if (state == MenuHandler::UNCHECKED)
      {
        menu_handler_.setCheckState(3, MenuHandler::UNCHECKED);
        menu_handler_.setCheckState(4, MenuHandler::CHECKED);
        menu_handler_.setCheckState(5, MenuHandler::UNCHECKED);
        menu_handler_.setCheckState(6, MenuHandler::UNCHECKED);
        if (show_description_control_)
        {
          removeDescriptionControl();
          description_ = tag_;
          addDescriptionControl();
        }
      }
      break;
    case 5:
      /*
       * Plane tag
       */
      tag_ = "Door";
      updatePublisher_->publishTagChanged(tag_);
      if (state == MenuHandler::UNCHECKED)
      {
        menu_handler_.setCheckState(3, MenuHandler::UNCHECKED);
        menu_handler_.setCheckState(4, MenuHandler::UNCHECKED);
        menu_handler_.setCheckState(5, MenuHandler::CHECKED);
        menu_handler_.setCheckState(6, MenuHandler::UNCHECKED);
        if (show_description_control_)
        {
          removeDescriptionControl();
          description_ = tag_;
          addDescriptionControl();
        }
      }
      break;
    case 6:
      /*
       * Plane tag
       */
      tag_ = "Table desk";
      updatePublisher_->publishTagChanged(tag_);
      if (state == MenuHandler::UNCHECKED)
      {
        menu_handler_.setCheckState(3, MenuHandler::UNCHECKED);
        menu_handler_.setCheckState(4, MenuHandler::UNCHECKED);
        menu_handler_.setCheckState(5, MenuHandler::UNCHECKED);
        menu_handler_.setCheckState(6, MenuHandler::CHECKED);
        if (show_description_control_)
        {
          removeDescriptionControl();
          description_ = tag_;
          addDescriptionControl();
        }
      }
      break;
  }

  server_->insert(object_);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

void Plane::createMenu()
{
  if (!menu_created_)
  {
    menu_created_ = true;
    menu_handler_.setCheckState(menu_handler_.insert("Show description", boost::bind(&Plane::menuCallback, this, _1)),
                               MenuHandler::UNCHECKED);
    MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Tag");
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Unknown", boost::bind(&Plane::menuCallback, this,
                                                                                           _1)), MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(
                               menu_handler_.insert(sub_menu_handle, "Wall", boost::bind(&Plane::menuCallback, this, _1)),
                               MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(
                               menu_handler_.insert(sub_menu_handle, "Door", boost::bind(&Plane::menuCallback, this, _1)),
                               MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Table desk", boost::bind(&Plane::menuCallback,
                                                                                              this, _1)),
                               MenuHandler::UNCHECKED);
  }
}

void Plane::create()
{
  clearObject();

  object_.header.frame_id = frame_id_;
  object_.header.stamp = ros::Time::now();
  object_.name = name_;
  object_.description = name_ + " plane";
  object_.pose = pose_;

  mesh_.type = Marker::CUBE;
  mesh_.color = color_;
  mesh_.scale = scale_;
  mesh_.scale.z = 0.001;

  control_.always_visible = true;
  control_.interaction_mode = InteractiveMarkerControl::BUTTON;
  control_.markers.push_back(mesh_);
  object_.controls.push_back(control_);

  createMenu();
}

void Plane::insert()
{
  create();

  server_->insert(object_);
  menu_handler_.apply(*server_, name_);
}

}
