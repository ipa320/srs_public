/******************************************************************************
 * \file
 *
 * $Id: Billboard.cpp 676 2012-04-19 18:32:07Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
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

#include <srs_interaction_primitives/billboard.h>

using namespace std;
using namespace interactive_markers;
using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std_msgs;

namespace srs_interaction_primitives
{

Billboard::Billboard(InteractiveMarkerServerPtr server, string frame_id, string name) :
    Primitive(server, frame_id, name, srs_interaction_primitives::PrimitiveType::BILLBOARD)
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
       * Billboard trajectory prediction
       */
      if (state == MenuHandler::CHECKED)
      {
        removeTrajectoryPredictionMarkers();
        menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
      }
      else
      {
        addTrajectoryPredictionMarkers();
        menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
      }
      break;
    case 3:
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

void Billboard::updateControls()
{
  if (show_trajectory_control_)
  {
    removeTrajectoryControls();
    addTrajectoryControls();
  }
  Primitive::updateControls();
}

void Billboard::removeTrajectoryControls()
{
  show_trajectory_control_ = false;
  removeControl("trajectory_control");
}

void Billboard::addTrajectoryControls()
{
  show_trajectory_control_ = true;

  trajectoryControl_.markers.clear();

  trajectoryControl_.name = "trajectory_control";
  trajectoryControl_.always_visible = true;
  trajectoryControl_.orientation_mode = InteractiveMarkerControl::FIXED;

  if (velocity_ != 0.0)
  {
    Marker trajectoryArrow;
    trajectoryArrow.type = Marker::ARROW;
    trajectoryArrow.pose.position.x = 0;
    trajectoryArrow.pose.position.y = 0;
    trajectoryArrow.pose.position.z = 0;
    trajectoryArrow.pose.orientation = direction_;
    trajectoryArrow.scale.x = 0.25;
    trajectoryArrow.scale.y = 0.25;
    trajectoryArrow.scale.z = velocity_;
    trajectoryArrow.color.r = 1.0;
    trajectoryArrow.color.g = 0.0;
    trajectoryArrow.color.b = 0.0;
    trajectoryArrow.color.a = 1.0;
    trajectoryControl_.markers.push_back(trajectoryArrow);
  }

  ostringstream velocityText;
  velocityText << setprecision(2) << fixed << velocity_ << "m/s";
  Marker trajectoryText;
  trajectoryText.type = Marker::TEXT_VIEW_FACING;
  trajectoryText.text = velocityText.str();
  trajectoryText.scale.z = 0.2;
  trajectoryText.color.r = 1.0;
  trajectoryText.color.g = 0.0;
  trajectoryText.color.b = 0.0;
  trajectoryText.color.a = 1.0;
  trajectoryText.pose.position.x = 0.0;
  trajectoryText.pose.position.y = 0.0;
  trajectoryText.pose.position.z = 0.0;
  trajectoryControl_.markers.push_back(trajectoryText);

  object_.controls.push_back(trajectoryControl_);
}

void Billboard::createMenu()
{
  if (!menu_created_)
  {
    menu_created_ = true;
    menu_handler_.setCheckState(
        menu_handler_.insert("Show trajectory", boost::bind(&Billboard::menuCallback, this, _1)),
        MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(
        menu_handler_.insert("Show trajectory prediction", boost::bind(&Billboard::menuCallback, this, _1)),
        MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(
        menu_handler_.insert("Show description", boost::bind(&Billboard::menuCallback, this, _1)),
        MenuHandler::UNCHECKED);
  }

}

void Billboard::createMesh()
{
  mesh_.type = Marker::MESH_RESOURCE;
  mesh_.mesh_use_embedded_materials = true;
  mesh_.scale.y = scale_.x;
  mesh_.scale.z = scale_.y;
  // Model flip
  mesh_.pose.orientation.x = 180;

  if (billboard_type_ == BillboardType::CHAIR)
    mesh_.mesh_resource = "package://srs_interaction_primitives/meshes/chair.dae";
  else if (billboard_type_ == BillboardType::MILK)
    mesh_.mesh_resource = "package://srs_interaction_primitives/meshes/milk.dae";
  else if (billboard_type_ == BillboardType::TABLE)
    mesh_.mesh_resource = "package://srs_interaction_primitives/meshes/table.dae";
  else if (billboard_type_ == BillboardType::PERSON)
    mesh_.mesh_resource = "package://srs_interaction_primitives/meshes/person.dae";
  else if (billboard_type_ == BillboardType::PERSON_HEAD)
    mesh_.mesh_resource = "package://srs_interaction_primitives/meshes/person_head.dae";
  else
    ROS_ERROR("UNKNOWN BILLBOARD TYPE!");
}

void Billboard::addTrajectoryPredictionMarkers()
{
  visualization_msgs::InteractiveMarker predictionMarker;
  predictionMarker = object_;
  predictionMarker.controls.clear();

  Ogre::Matrix3 *rotation = new Ogre::Matrix3();
  Ogre::Quaternion orientation;
  orientation.x = direction_.x;
  orientation.y = direction_.y;
  orientation.z = direction_.z;
  orientation.w = direction_.w;
  orientation.normalise();
  orientation.ToRotationMatrix(*rotation);
  Ogre::Vector3 position;
  position.x = velocity_;
  position.y = 0;
  position.z = 0;
  position = rotation->operator *(position);

  predictionMarker.pose.position.x = pose_.position.x;
  predictionMarker.pose.position.y = pose_.position.y;
  predictionMarker.pose.position.z = pose_.position.z;

  visualization_msgs::InteractiveMarkerControl predictionControl;
  predictionControl.name = "prediction_control";
  predictionControl.always_visible = true;
  predictionControl.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  predictionControl.interaction_mode = InteractiveMarkerControl::NONE;

  for (int i = 1; i < PREDICTIONS_COUNT + 1; i++)
  {
    std::stringstream name;
    name << name_ << "_prediction_" << i;
    predictionMarker.name = name.str();
    std::stringstream desc;
    desc << i << "s";
    predictionMarker.description = desc.str();
    predictionMarker.pose.position.x += position.x;
    predictionMarker.pose.position.y += position.y;
    predictionMarker.pose.position.z += position.z;

    predictionControl.markers.clear();
    predictionMarker.controls.clear();

    visualization_msgs::Marker sphere;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.color.g = 1;
    sphere.color.b = 1;
    sphere.color.a = 1;
    sphere.scale.x = PREDICTION_SPHERE_SIZE;
    sphere.scale.y = PREDICTION_SPHERE_SIZE;
    sphere.scale.z = PREDICTION_SPHERE_SIZE;

    predictionControl.markers.push_back(sphere);
    predictionMarker.controls.push_back(predictionControl);

    server_->insert(predictionMarker);
  }
  server_->applyChanges();
}

void Billboard::removeTrajectoryPredictionMarkers()
{
  for (int i = 1; i < PREDICTIONS_COUNT + 1; i++)
  {
    std::stringstream name;
    name << name_ << "_prediction_" << i;
    server_->erase(name.str());
  }
  server_->applyChanges();
}

void Billboard::create()
{
  clearObject();

  object_.header.frame_id = frame_id_;
  object_.name = name_;
//  object_.description = name_ + " billboard";
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

