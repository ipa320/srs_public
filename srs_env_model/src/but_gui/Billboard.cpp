/******************************************************************************
 * \file
 *
 * $Id: Billboard.cpp 603 2012-04-16 10:50:03Z xlokaj03 $
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

#include "but_gui/Billboard.h"

namespace but_gui
{

Billboard::Billboard(InteractiveMarkerServerPtr server, string frame_id, string name) :
  Primitive(server, frame_id, name, srs_env_model::PrimitiveType::BILLBOARD)
{
  velocity_ = 0.0;
  color_.r = 0;
  color_.g = 0;
  color_.b = 1;
  color_.a = 1;
}

void Billboard::menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
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
       * Billboard trajectory
       */
      if (state == MenuHandler::CHECKED)
      {
        removeTrajectoryControls();
        menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
      }
      else
      {
        addTrajectoryControls();
        menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
      }
      break;
    case 2:
      /*
       * Billboard description
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
  }

  server_->insert(object_);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

void Billboard::setType(int type)
{
  billboard_type_ = type;
}

int Billboard::getType()
{
  return billboard_type_;
}

void Billboard::createMenu()
{
  if (!menu_created_)
  {
    menu_created_ = true;
    menu_handler_.setCheckState(
                                menu_handler_.insert("Show trajectory", boost::bind(&Billboard::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert("Show description",
                                                     boost::bind(&Billboard::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);
  }

}

void Billboard::createMesh()
{
  mesh_.type = Marker::MESH_RESOURCE;
  mesh_.mesh_use_embedded_materials = true;
  mesh_.scale.y = scale_.x;
  mesh_.scale.z = scale_.y;
  mesh_.pose.orientation.x = 180;

  if (billboard_type_ == BillboardType::CHAIR)
    mesh_.mesh_resource = "package://srs_env_model/meshes/chair.dae";
  else if (billboard_type_ == BillboardType::MILK)
    mesh_.mesh_resource = "package://srs_env_model/meshes/milk.dae";
  else if (billboard_type_ == BillboardType::TABLE)
    mesh_.mesh_resource = "package://srs_env_model/meshes/table.dae";
  else if (billboard_type_ == BillboardType::PERSON)
    mesh_.mesh_resource = "package://srs_env_model/meshes/person.dae";
  else if (billboard_type_ == BillboardType::PERSON_HEAD)
    mesh_.mesh_resource = "package://srs_env_model/meshes/person_head.dae";
  else
    ROS_ERROR("UNKNOWN BILLBOARD TYPE!");
}

void Billboard::create()
{
  clearObject();

  object_.header.frame_id = frame_id_;
  object_.header.stamp = ros::Time::now();
  object_.name = name_;
  object_.description = name_ + " billboard";
  object_.pose.position.x = pose_.position.x;
  object_.pose.position.y = pose_.position.y;
  object_.pose.position.z = pose_.position.z;

  createMesh();

  control_.name = "billboard_control";
  control_.always_visible = true;
  control_.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control_.interaction_mode = InteractiveMarkerControl::BUTTON;
  control_.markers.push_back(mesh_);
  object_.controls.push_back(control_);

  createMenu();
}

void Billboard::insert()
{
  create();

  server_->insert(object_, boost::bind(&Primitive::defaultCallback, this, _1));
  menu_handler_.apply(*server_, name_);
}

}

