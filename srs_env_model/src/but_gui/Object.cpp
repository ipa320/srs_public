/******************************************************************************
 * \file
 *
 * $Id: Object.cpp 603 2012-04-16 10:50:03Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 20/1/2012
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

#include <but_gui/Object.h>

namespace but_gui
{

Object::Object(InteractiveMarkerServerPtr server, string frame_id, string name) :
  Primitive(server, frame_id, name, srs_env_model::PrimitiveType::OBJECT)
{
  description_ = "";
  use_material_ = false;
  pregrasp1_.name = "control_grasp_1";
  pregrasp2_.name = "control_grasp_2";
  pregrasp3_.name = "control_grasp_3";
  pregrasp4_.name = "control_grasp_4";
  pregrasp5_.name = "control_grasp_5";
  pregrasp6_.name = "control_grasp_6";
}

void Object::objectCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (show_scale_control_)
  {
    updateScaleControls();
    server_->applyChanges();
  }
  defaultCallback(feedback);
}

void Object::menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  ROS_INFO("MENU CLICKED");
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::EntryHandle moveToPreGraspHandle = 8;
  MenuHandler::CheckState state;
  string title;
  menu_handler_.getCheckState(handle, state);
  menu_handler_.getTitle(handle, title);

  updatePublisher_->publishMenuClicked(title, state);

  InteractiveMarker o;
  if (server_->get(name_, o))
  {
    object_.pose = o.pose;
    switch (feedback->menu_entry_id)
    {
      case 1:
        /*
         * Object description
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
      case 2:
        /*
         * Object measure
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
      case 3:
        /*
         * Object pre-grasp positions
         */
        if (state == MenuHandler::CHECKED)
        {
          menu_handler_.setVisible(moveToPreGraspHandle, false);
          removePregraspPositions();
          menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          menu_handler_.setVisible(moveToPreGraspHandle, true);
          addPregraspPositions();
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
      case 7:
        /*
         * Scale controls
         */
        if (state == MenuHandler::CHECKED)
        {
          removeScaleControls();
          menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          addScaleControls();
          menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 8:
        /*
         * Move arm to pre-grasp position
         */
        cout << feedback->control_name << endl;
        break;
    }
    server_->insert(object_);
  }

  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

void Object::createMenu()
{
  if (!menu_created_)
  {
    menu_created_ = true;
    menu_handler_.setCheckState(menu_handler_.insert("Show description", boost::bind(&Object::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert("Show measure", boost::bind(&Object::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert("Show pre-grasp positions", boost::bind(&Object::menuCallback,
                                                                                             this, _1)),
                                MenuHandler::UNCHECKED);

    MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Interaction");
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Movement", boost::bind(&Object::menuCallback,
                                                                                              this, _1)),
                                MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Rotation", boost::bind(&Object::menuCallback,
                                                                                              this, _1)),
                                MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Scale", boost::bind(&Object::menuCallback, this,
                                                                                           _1)), MenuHandler::UNCHECKED);
    menu_handler_.setVisible(menu_handler_.insert("Move arm to pre-grasp position", boost::bind(&Object::menuCallback,
                                                                                                this, _1)), false);
  }
}

void Object::createBox()
{
  mesh_.type = Marker::MESH_RESOURCE;
  mesh_.color = color_;
  mesh_.mesh_use_embedded_materials = use_material_;
  mesh_.scale = scale_;
  mesh_.mesh_resource = resource_;
}

void Object::createObjectBox()
{
  object_.header.frame_id = frame_id_;
  object_.header.stamp = ros::Time::now();
  object_.name = name_;
  object_.description = description_;
  object_.pose = pose_;
  object_.scale = but_gui::maxScale(scale_);
}

void Object::create()
{
  clearObject();

  createObjectBox();

  control_.name = "box_control";
  createBox();
  control_.markers.push_back(mesh_);
  control_.interaction_mode = InteractiveMarkerControl::MENU;
  control_.always_visible = true;
  object_.controls.push_back(control_);

  createMenu();
}

void Object::insert()
{
  create();

  server_->insert(object_, boost::bind(&Object::objectCallback, this, _1));
  menu_handler_.apply(*server_, name_);
}

}
