/******************************************************************************
 * \file
 *
 * $Id: UnknownObject.cpp 676 2012-04-19 18:32:07Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 21/12/2011
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

#include <srs_interaction_primitives/unknown_object.h>

using namespace std;
using namespace interactive_markers;
using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std_msgs;

namespace srs_interaction_primitives
{

UnknownObject::UnknownObject(InteractiveMarkerServerPtr server, string frame_id, string name)
    : Primitive(server, frame_id, name, srs_interaction_primitives::PrimitiveType::UNKNOWN_OBJECT)
{
	description_ = "";

//	color_.r = 0.3;
//	color_.g = 0.5;
//	color_.b = 0.6;
//	color_.a = 1.0;
	color_.r = 0.85;
	color_.g = 0.85;
	color_.b = 0.9;
	color_.a = 0.7;

	show_movement_control_ = show_scale_control_ = show_rotation_control_ = show_measure_control_ = show_description_control_ = false;
	use_material_ = true;
	allow_object_interaction_ = false;
}


void UnknownObject::setAllowObjectInteraction(bool allow)
{
  ROS_INFO("Interaction allowed");

  allow_object_interaction_ = allow;
  if (allow_object_interaction_)
  {
    addMovementControls();
    addRotationControls();
    addScaleControls();

    menu_handler_.setCheckState(menu_handler_interaction_movement_, MenuHandler::CHECKED);
    menu_handler_.setCheckState(menu_handler_interaction_rotation_, MenuHandler::CHECKED);
    menu_handler_.setCheckState(menu_handler_interaction_scale_, MenuHandler::CHECKED);
  }
  else
  {
    removeMovementControls();
    removeRotationControls();
    removeScaleControls();
  
    menu_handler_.setCheckState(menu_handler_interaction_movement_, MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_interaction_rotation_, MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_interaction_scale_, MenuHandler::UNCHECKED);
  }

  server_->insert(object_);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}


void UnknownObject::uboxCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  defaultCallback(feedback);

  if (show_scale_control_)
  {
    updateScaleControls();
    server_->applyChanges();
  }
}


void UnknownObject::menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
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
      /**
       * Uknown object description
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
      /**
       * Unknown object measure
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

    case 4:
      /*
       * Enable all interaction controls
       */
      addMovementControls();
      addRotationControls();
      addScaleControls();
      menu_handler_.setCheckState(menu_handler_interaction_movement_, MenuHandler::CHECKED);
      menu_handler_.setCheckState(menu_handler_interaction_rotation_, MenuHandler::CHECKED);
      menu_handler_.setCheckState(menu_handler_interaction_scale_, MenuHandler::CHECKED);
      break;
    case 5:
      /*
       * Disable all interaction controls
       */
      removeMovementControls();
      removeRotationControls();
      removeScaleControls();
      menu_handler_.setCheckState(menu_handler_interaction_movement_, MenuHandler::UNCHECKED);
      menu_handler_.setCheckState(menu_handler_interaction_rotation_, MenuHandler::UNCHECKED);
      menu_handler_.setCheckState(menu_handler_interaction_scale_, MenuHandler::UNCHECKED);
      break;

    case 6:
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
    case 7:
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
  }

  server_->insert(object_);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

void UnknownObject::createMenu()
{
  if (!menu_created_)
  {
    menu_created_ = true;
    menu_handler_.setCheckState(
        menu_handler_.insert("Show Description", boost::bind(&UnknownObject::menuCallback, this, _1)),
        MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(
        menu_handler_.insert("Show Measure", boost::bind(&UnknownObject::menuCallback, this, _1)),
        MenuHandler::UNCHECKED);

    menu_handler_interaction_ = menu_handler_.insert("Interaction");
    menu_handler_.setCheckState(
        menu_handler_.insert(menu_handler_interaction_, "Enable All", boost::bind(&UnknownObject::menuCallback, this, _1)),
        MenuHandler::NO_CHECKBOX);
    menu_handler_.setCheckState(
        menu_handler_.insert(menu_handler_interaction_, "Disable All", boost::bind(&UnknownObject::menuCallback, this, _1)),
        MenuHandler::NO_CHECKBOX);

    menu_handler_interaction_movement_ = menu_handler_.insert(menu_handler_interaction_, "Movement",
                                                              boost::bind(&UnknownObject::menuCallback, this, _1));
    menu_handler_interaction_rotation_ = menu_handler_.insert(menu_handler_interaction_, "Rotation",
                                                              boost::bind(&UnknownObject::menuCallback, this, _1));
    menu_handler_interaction_scale_    = menu_handler_.insert(menu_handler_interaction_, "Scale",
                                                              boost::bind(&UnknownObject::menuCallback, this, _1));

    menu_handler_.setCheckState(menu_handler_interaction_movement_, MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_interaction_rotation_, MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_interaction_scale_, MenuHandler::UNCHECKED);
  }
}


void UnknownObject::createBox()
{
  box_.type = Marker::MESH_RESOURCE;
  box_.mesh_use_embedded_materials = true;
  box_.scale = scale_;
  box_.mesh_resource = "package://srs_interaction_primitives/meshes/unknown_object.dae";
}


void UnknownObject::createColorBox()
{
  // Transparent box
  box_.type = Marker::CUBE;
  box_.pose.position.x = 0;
  box_.pose.position.y = 0;
  box_.pose.position.z = 0;
  box_.scale = scale_;
  box_.color = color_;
//  box_.color.a = 0.5;
  box_.color.a = 0.75;

  // Wireframe model - disabled for now, it doesn't scale properly...
  wire_.points.clear();

  Point p1, p2;
  double sx = scale_.x / 2;
  double sy = scale_.y / 2;
  double sz = scale_.z / 2;
  double trans_x = 0;
  double trans_y = 0;
  double trans_z = 0;

  wire_.type = Marker::LINE_LIST;
  wire_.pose.position.x = 0;
  wire_.pose.position.y = 0;
  wire_.pose.position.z = 0;
  wire_.scale = scale_;
  wire_.color = color_;
  wire_.color.a = 1.0;
//  wire_.scale.x = 0.002;
  wire_.scale.x = 0.004;

  p1.x = -sx + trans_x;
  p1.y = -sy + trans_y;
  p1.z = -sz + trans_z;
  p2.x = -sx + trans_x;
  p2.y = sy + trans_y;
  p2.z = -sz + trans_z;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);
  p2.x = -sx + trans_x;
  p2.y = -sy + trans_y;
  p2.z = sz + trans_z;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);
  p1.x = -sx + trans_x;
  p1.y = sy + trans_y;
  p1.z = sz + trans_z;
  p2.x = -sx + trans_x;
  p2.y = sy + trans_y;
  p2.z = -sz + trans_z;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);
  p2.x = -sx + trans_x;
  p2.y = -sy + trans_y;
  p2.z = sz + trans_z;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);

  p1.x = sx + trans_x;
  p1.y = -sy + trans_y;
  p1.z = -sz + trans_z;
  p2.x = sx + trans_x;
  p2.y = sy + trans_y;
  p2.z = -sz + trans_z;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);
  p2.x = sx + trans_x;
  p2.y = -sy + trans_y;
  p2.z = sz + trans_z;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);
  p1.x = sx + trans_x;
  p1.y = sy + trans_y;
  p1.z = sz + trans_z;
  p2.x = sx + trans_x;
  p2.y = sy + trans_y;
  p2.z = -sz + trans_z;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);
  p2.x = sx + trans_x;
  p2.y = -sy + trans_y;
  p2.z = sz + trans_z;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);

  p1.x = sx + trans_x;
  p1.y = sy + trans_y;
  p1.z = sz + trans_z;
  p2.x = -sx + trans_x;
  p2.y = sy + trans_y;
  p2.z = sz + trans_z;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);

  p1.x = sx + trans_x;
  p1.y = -sy + trans_y;
  p1.z = sz + trans_z;
  p2.x = -sx + trans_x;
  p2.y = -sy + trans_y;
  p2.z = sz + trans_z;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);

  p1.x = sx + trans_x;
  p1.y = sy + trans_y;
  p1.z = -sz + trans_z;
  p2.x = -sx + trans_x;
  p2.y = sy + trans_y;
  p2.z = -sz + trans_z;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);

  p1.x = sx + trans_x;
  p1.y = -sy + trans_y;
  p1.z = -sz + trans_z;
  p2.x = -sx + trans_x;
  p2.y = -sy + trans_y;
  p2.z = -sz + trans_z;
  wire_.points.push_back(p1);
  wire_.points.push_back(p2);
}


void UnknownObject::createUnknownBox()
{
  object_.header.frame_id = frame_id_;
  object_.name = name_;
//  object_.description = name_;
  object_.pose = pose_;
  object_.scale = srs_interaction_primitives::maxScale(scale_);
}


void UnknownObject::create()
{
  clearObject();

  createUnknownBox();

  control_.name = "box_control";
  if( use_material_ )
  {
      createBox();
      control_.markers.push_back(box_);
  }
  else
  {
      createColorBox();
      control_.markers.push_back(box_);
      control_.markers.push_back(wire_);
  }
  control_.interaction_mode = InteractiveMarkerControl::MENU;
  control_.always_visible = true;
  object_.controls.push_back(control_);

  createMenu();
}

void UnknownObject::insert()
{
  create();

  server_->insert(object_, boost::bind(&UnknownObject::uboxCallback, this, _1));
  menu_handler_.apply(*server_, name_);
}

}
