/******************************************************************************
 * \file
 *
 * $Id: Object.cpp 676 2012-04-19 18:32:07Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 24/2/2012
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

#include <srs_interaction_primitives/object.h>

using namespace std;
using namespace interactive_markers;
using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std_msgs;


namespace srs_interaction_primitives
{

Object::Object(InteractiveMarkerServerPtr server, string frame_id, string name) :
  BoundingBox(server, frame_id, name)
{
  setPrimitiveType(srs_interaction_primitives::PrimitiveType::OBJECT);
  use_material_ = translated_ = false;
  move_arm_to_pregrasp_onclick_ = false;
  pregrasp1_.name = "control_grasp_xp";
  pregrasp2_.name = "control_grasp_xm";
  pregrasp3_.name = "control_grasp_yp";
  pregrasp4_.name = "control_grasp_ym";
  pregrasp5_.name = "control_grasp_zp";
  pregrasp6_.name = "control_grasp_zm";
}

void Object::objectWithBoundingBoxCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (move_arm_to_pregrasp_onclick_ && feedback->event_type == InteractiveMarkerFeedback::MOUSE_DOWN)
  {
    if (feedback->control_name == pregrasp1_.name)
      updatePublisher_->publishMoveArmToPreGrasp(1);
    else if (feedback->control_name == pregrasp2_.name)
      updatePublisher_->publishMoveArmToPreGrasp(2);
    else if (feedback->control_name == pregrasp3_.name)
      updatePublisher_->publishMoveArmToPreGrasp(3);
    else if (feedback->control_name == pregrasp4_.name)
      updatePublisher_->publishMoveArmToPreGrasp(4);
    else if (feedback->control_name == pregrasp5_.name)
      updatePublisher_->publishMoveArmToPreGrasp(5);
    else if (feedback->control_name == pregrasp6_.name)
      updatePublisher_->publishMoveArmToPreGrasp(6);
  }

  defaultCallback(feedback);
}

void Object::menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  string title;
  menu_handler_.getCheckState(handle, state);
  menu_handler_.getTitle(handle, title);

  updatePublisher_->publishMenuClicked(title, state);

  /*InteractiveMarker o;
   if (server_->get(name_, o))
   {
   pose_ = o.pose;
   object_.pose = pose_;
   }*/

  switch (feedback->menu_entry_id)
  {
    case 1:
      /*
       * Object's bounding box visibility
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
       * Object's description visibility
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
       * Object's measure visibility
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
       * Object's pre-grasp positions visibility
       */
      if (state == MenuHandler::CHECKED)
      {
        removePregraspPositions();
        menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
      }
      else
      {
        addPregraspPositions();
        menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
      }
      break;
    case 5:
      /*
       * Move arm to pre-grasp position on click
       */
      if (state == MenuHandler::CHECKED)
      {
        move_arm_to_pregrasp_onclick_ = false;
        menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
      }
      else
      {
        move_arm_to_pregrasp_onclick_ = true;
        menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
      }
      break;
    case 7:
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
    case 8:
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
  }
  /*
   // Transfer object into IMS frame
   if (feedback->header.frame_id != frame_id_)
   {
   frame_id_ = feedback->header.frame_id;
   object_.header.frame_id = frame_id_;
   }
   */
  server_->insert(object_);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

void Object::createMenu()
{
  if (!menu_created_)
  {
    menu_created_ = true;
    menu_handler_.setCheckState(
                                menu_handler_.insert("Show bounding box", boost::bind(&Object::menuCallback, this, _1)),
                                MenuHandler::CHECKED);
    menu_handler_.setCheckState(menu_handler_.insert("Show description", boost::bind(&Object::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert("Show measure", boost::bind(&Object::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);

    bool show_pregrasp = false;
    if (ros::param::has(PARAMETER_SHOW_PREGRASP))
      ros::param::get(PARAMETER_SHOW_PREGRASP, show_pregrasp);
    int handle_pregrasp =
        menu_handler_.insert("Show pre-grasp positions", boost::bind(&Object::menuCallback, this, _1));
    if (show_pregrasp)
    {
      menu_handler_.setCheckState(handle_pregrasp, MenuHandler::CHECKED);
      addPregraspPositions();
    }
    else
      menu_handler_.setCheckState(handle_pregrasp, MenuHandler::UNCHECKED);

    menu_handler_.setCheckState(menu_handler_.insert("Move arm to pre-grasp position on click",
                                                     boost::bind(&Object::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);

    bool allow_interation = true;
    if (ros::param::has(PARAMETER_ALLOW_INTERACTION))
      ros::param::get(PARAMETER_ALLOW_INTERACTION, allow_interation);

    if (allow_interation)
    {
      MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Interaction");
      menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Movement", boost::bind(&Object::menuCallback,
                                                                                                this, _1)),
                                  MenuHandler::UNCHECKED);
      menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Rotation", boost::bind(&Object::menuCallback,
                                                                                                this, _1)),
                                  MenuHandler::UNCHECKED);
    }
  }
}

void Object::createMesh()
{
  mesh_.pose = pose_;
  mesh_.header.frame_id = frame_id_;
  mesh_.pose.position.z -= 0.5 * bounding_box_lwh_.z;

  mesh_.color = color_;
  mesh_.scale = Vector3();
  switch (object_resource_)
  {
    case RESOURCE_FILE:
      mesh_.type = Marker::MESH_RESOURCE;
      mesh_.mesh_use_embedded_materials = use_material_;
      mesh_.mesh_resource = resource_;
      break;
    case SHAPE:
      mesh_.type = Marker::TRIANGLE_LIST;
      for (unsigned int i = 0; i < shape_.triangles.size(); i++)
        mesh_.points.push_back(shape_.vertices[shape_.triangles[i]]);
      break;
  }
}

void Object::setPoseLWH(Pose pose, Point bounding_box_lwh)
{
  bounding_box_lwh_ = bounding_box_lwh;

  pose_.position.z += 0.5 * bounding_box_lwh.z;
  BoundingBox::setPose(pose);
}

void Object::create()
{
  clearObject();

  scale_.x = bounding_box_lwh_.x;
  scale_.y = bounding_box_lwh_.y;
  //  scale_.z = 0.5 * bounding_box_lwh_.z;
  scale_.z = bounding_box_lwh_.z;

  object_.header.frame_id = frame_id_;
  object_.header.stamp = ros::Time::now();
  object_.name = name_;
  object_.description = description_;
  object_.pose = pose_;
  object_.scale = maxScale(scale_);

  //  if (!translated_)
  //  {
  //    if (frame_id_ == "/map")
  //      pose_.position.z += scale_.z / 2;
  //    else
  //      pose_.position.y += scale_.z / 2;
  //    translated_ = true;
  //  }

  BoundingBox::createBoundingBoxControl();
  createMesh();
  control_.markers.push_back(mesh_);
  object_.controls.clear();
  object_.controls.push_back(control_);

  createMenu();
}

void Object::insert()
{
  create();
  updateControls();
  server_->insert(object_, boost::bind(&Object::objectWithBoundingBoxCallback, this, _1));
  menu_handler_.apply(*server_, name_);
}

}
