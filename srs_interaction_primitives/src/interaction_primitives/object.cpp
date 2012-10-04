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
    BoundingBox(server, frame_id, name), show_pregrasp_control_(false), use_material_(false), translated_(false), allow_object_interaction_(
        true)
{
  setPrimitiveType(srs_interaction_primitives::PrimitiveType::OBJECT);

  if (ros::param::has(MoveArmToPregraspOnClick_PARAM))
    ros::param::get(MoveArmToPregraspOnClick_PARAM, move_arm_to_pregrasp_onclick_);
  else
    move_arm_to_pregrasp_onclick_ = false;

  pregrasp1_.name = "control_grasp_xp";
  pregrasp2_.name = "control_grasp_xm";
  pregrasp3_.name = "control_grasp_yp";
  pregrasp4_.name = "control_grasp_ym";
  pregrasp5_.name = "control_grasp_zp";
  pregrasp6_.name = "control_grasp_zm";

  if (pose_type_ == PoseType::POSE_BASE)
  {
    pose_.position.z += scale_.z * 0.5;
  }
}

void Object::objectCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
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

  server_->insert(object_);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

void Object::createMenu()
{
  if (!menu_created_)
  {
    menu_created_ = true;
    menu_handler_.setCheckState(menu_handler_.insert("Show bounding box", boost::bind(&Object::menuCallback, this, _1)),
                                MenuHandler::CHECKED);
    menu_handler_.setCheckState(menu_handler_.insert("Show description", boost::bind(&Object::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert("Show measure", boost::bind(&Object::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);

    bool show_pregrasp = false;
    if (ros::param::has(ShowPregrasp_PARAM))
      ros::param::get(ShowPregrasp_PARAM, show_pregrasp);
    menu_handler_show_pregrasp_ = menu_handler_.insert("Show pre-grasp positions",
                                                       boost::bind(&Object::menuCallback, this, _1));
    if (show_pregrasp)
    {
      menu_handler_.setCheckState(menu_handler_show_pregrasp_, MenuHandler::CHECKED);
      addPregraspPositions();
    }
    else
      menu_handler_.setCheckState(menu_handler_show_pregrasp_, MenuHandler::UNCHECKED);

    menu_handler_move_to_pregrasp_ = menu_handler_.insert("Move arm to pre-grasp position on click",
                                                          boost::bind(&Object::menuCallback, this, _1));
    if (move_arm_to_pregrasp_onclick_)
      menu_handler_.setCheckState(menu_handler_move_to_pregrasp_, MenuHandler::CHECKED);
    else
      menu_handler_.setCheckState(menu_handler_move_to_pregrasp_, MenuHandler::UNCHECKED);

    /*    if (ros::param::has (AllowInteraction_PARAM))
     ros::param::get(AllowInteraction_PARAM, allow_interation_);*/

    /* if (allow_object_interaction_)
     {
     MenuHandler::EntryHandle menu_handler_interaction_ = menu_handler_.insert("Interaction");
     menu_handler_.setCheckState(
     menu_handler_.insert(menu_handler_interaction_, "Movement", boost::bind(&Object::menuCallback, this, _1)),
     MenuHandler::UNCHECKED);
     menu_handler_.setCheckState(
     menu_handler_.insert(menu_handler_interaction_, "Rotation", boost::bind(&Object::menuCallback, this, _1)),
     MenuHandler::UNCHECKED);
     }*/
    menu_handler_interaction_ = menu_handler_.insert("Interaction");
    menu_handler_interaction_movement_ = menu_handler_.insert(menu_handler_interaction_, "Movement",
                                                              boost::bind(&Object::menuCallback, this, _1));
    menu_handler_interaction_rotation_ = menu_handler_.insert(menu_handler_interaction_, "Rotation",
                                                              boost::bind(&Object::menuCallback, this, _1));

    menu_handler_.setCheckState(menu_handler_interaction_movement_, MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_interaction_rotation_, MenuHandler::UNCHECKED);

    if (!allow_object_interaction_)
    {
      menu_handler_.setVisible(menu_handler_interaction_, false);
      menu_handler_.setVisible(menu_handler_interaction_movement_, false);
      menu_handler_.setVisible(menu_handler_interaction_rotation_, false);
    }
    if (!allow_pregrasp_)
    {
      menu_handler_.setVisible(menu_handler_show_pregrasp_, false);
      menu_handler_.setVisible(menu_handler_move_to_pregrasp_, false);
    }
  }
}

void Object::addPregraspPositions()
{
  show_pregrasp_control_ = true;

  Marker arrow;
  arrow.color = color_green_a01_;
  arrow.color.a = GRASP_TRANSPARENCY;
  arrow.scale.x = GRASP_ARROW_LENGTH;
  arrow.scale.y = GRASP_ARROW_WIDTH;
  arrow.scale.z = GRASP_ARROW_WIDTH;

  Marker text;
  text.type = Marker::TEXT_VIEW_FACING;
  text.scale.z = GRASP_TEXT_SIZE;
  text.color = color_green_a01_;
  text.color.a = GRASP_TRANSPARENCY;

  Marker point;
  point.type = Marker::SPHERE;
  point.scale.x = GRASP_POINT_SCALE;
  point.scale.y = GRASP_POINT_SCALE;
  point.scale.z = GRASP_POINT_SCALE;
  point.color.r = 1;
  point.color.g = 1;
  point.color.b = 0;
  point.color.a = arrow.color.a = GRASP_TRANSPARENCY;

  pregrasp1Control_.markers.clear();
  pregrasp2Control_.markers.clear();
  pregrasp3Control_.markers.clear();
  pregrasp4Control_.markers.clear();
  pregrasp5Control_.markers.clear();
  pregrasp6Control_.markers.clear();

  if (pregrasp1_.enabled)
  {
    pregrasp1Control_.name = pregrasp1_.name;
    pregrasp1Control_.interaction_mode = InteractiveMarkerControl::BUTTON;

    arrow.pose = pregrasp1_.pose;
    pregrasp1Control_.markers.push_back(arrow);

    text.pose.position.x = pregrasp1_.pose.position.x;
    text.pose.position.y = pregrasp1_.pose.position.y;
    text.pose.position.z = pregrasp1_.pose.position.z + GRASP_TEXT_OFFSET;
    text.text = "1";
    pregrasp1Control_.markers.push_back(text);

    point.pose = pregrasp1_.pose;
    pregrasp1Control_.markers.push_back(point);

    object_.controls.push_back(pregrasp1Control_);
  }

  if (pregrasp2_.enabled)
  {
    pregrasp2Control_.name = pregrasp2_.name;
    pregrasp2Control_.interaction_mode = InteractiveMarkerControl::BUTTON;

    arrow.pose = pregrasp2_.pose;
    pregrasp2Control_.markers.push_back(arrow);

    text.pose.position.x = pregrasp2_.pose.position.x;
    text.pose.position.y = pregrasp2_.pose.position.y;
    text.pose.position.z = pregrasp2_.pose.position.z + GRASP_TEXT_OFFSET;
    text.text = "2";
    pregrasp2Control_.markers.push_back(text);

    point.pose = pregrasp2_.pose;
    pregrasp2Control_.markers.push_back(point);

    object_.controls.push_back(pregrasp2Control_);
  }

  if (pregrasp3_.enabled)
  {
    pregrasp3Control_.name = pregrasp3_.name;
    pregrasp3Control_.interaction_mode = InteractiveMarkerControl::BUTTON;

    arrow.pose = pregrasp3_.pose;
    pregrasp3Control_.markers.push_back(arrow);

    text.pose.position.x = pregrasp3_.pose.position.x;
    text.pose.position.y = pregrasp3_.pose.position.y;
    text.pose.position.z = pregrasp3_.pose.position.z + GRASP_TEXT_OFFSET;
    text.text = "3";
    pregrasp3Control_.markers.push_back(text);

    point.pose = pregrasp3_.pose;
    pregrasp3Control_.markers.push_back(point);

    object_.controls.push_back(pregrasp3Control_);
  }

  if (pregrasp4_.enabled)
  {
    pregrasp4Control_.name = pregrasp4_.name;
    pregrasp4Control_.interaction_mode = InteractiveMarkerControl::BUTTON;

    arrow.pose = pregrasp4_.pose;
    pregrasp4Control_.markers.push_back(arrow);

    text.pose.position.x = pregrasp4_.pose.position.x;
    text.pose.position.y = pregrasp4_.pose.position.y;
    text.pose.position.z = pregrasp4_.pose.position.z + GRASP_TEXT_OFFSET;
    text.text = "4";
    pregrasp4Control_.markers.push_back(text);

    point.pose = pregrasp4_.pose;
    pregrasp4Control_.markers.push_back(point);

    object_.controls.push_back(pregrasp4Control_);
  }

  if (pregrasp5_.enabled)
  {
    pregrasp5Control_.name = pregrasp5_.name;
    pregrasp5Control_.interaction_mode = InteractiveMarkerControl::BUTTON;

    arrow.pose = pregrasp5_.pose;
    pregrasp5Control_.markers.push_back(arrow);

    text.pose.position.x = pregrasp5_.pose.position.x;
    text.pose.position.y = pregrasp5_.pose.position.y;
    text.pose.position.z = pregrasp5_.pose.position.z + GRASP_TEXT_OFFSET;
    text.text = "5";
    pregrasp5Control_.markers.push_back(text);

    point.pose = pregrasp5_.pose;
    pregrasp5Control_.markers.push_back(point);

    object_.controls.push_back(pregrasp5Control_);
  }

  if (pregrasp6_.enabled)
  {
    pregrasp6Control_.name = pregrasp6_.name;
    pregrasp6Control_.interaction_mode = InteractiveMarkerControl::BUTTON;

    arrow.pose = pregrasp6_.pose;
    pregrasp6Control_.markers.push_back(arrow);

    text.pose.position.x = pregrasp6_.pose.position.x;
    text.pose.position.y = pregrasp6_.pose.position.y;
    text.pose.position.z = pregrasp6_.pose.position.z + GRASP_TEXT_OFFSET;
    text.text = "6";
    pregrasp6Control_.markers.push_back(text);

    point.pose = pregrasp6_.pose;
    pregrasp6Control_.markers.push_back(point);

    object_.controls.push_back(pregrasp6Control_);
  }

}

void Object::removePregraspPositions()
{
  show_pregrasp_control_ = false;
  removeControl(pregrasp1_.name);
  removeControl(pregrasp2_.name);
  removeControl(pregrasp3_.name);
  removeControl(pregrasp4_.name);
  removeControl(pregrasp5_.name);
  removeControl(pregrasp6_.name);
}

void Object::addPreGraspPosition(int pos_id, Pose pose)
{
  switch (pos_id)
  {
    case GRASP_1:
      pregrasp1_.enabled = true;
      pregrasp1_.pose = pose;
      break;
    case GRASP_2:
      pregrasp2_.enabled = true;
      pregrasp2_.pose = pose;
      break;
    case GRASP_3:
      pregrasp3_.enabled = true;
      pregrasp3_.pose = pose;
      break;
    case GRASP_4:
      pregrasp4_.enabled = true;
      pregrasp4_.pose = pose;
      break;
    case GRASP_5:
      pregrasp5_.enabled = true;
      pregrasp5_.pose = pose;
      break;
    case GRASP_6:
      pregrasp6_.enabled = true;
      pregrasp6_.pose = pose;
      break;
    default:
      ROS_WARN("Unknown pre-grasp position");
      return;
      break;
  }
}

void Object::updateControls()
{
  if (show_pregrasp_control_)
  {
    removePregraspPositions();
    addPregraspPositions();
  }
  Primitive::updateControls();
}

void Object::removePreGraspPosition(int pos_id)
{
  switch (pos_id)
  {
    case GRASP_1:
      pregrasp1_.enabled = false;
      break;
    case GRASP_2:
      pregrasp2_.enabled = false;
      break;
    case GRASP_3:
      pregrasp3_.enabled = false;
      break;
    case GRASP_4:
      pregrasp4_.enabled = false;
      break;
    case GRASP_5:
      pregrasp5_.enabled = false;
      break;
    case GRASP_6:
      pregrasp6_.enabled = false;
      break;
    default:
      ROS_WARN("Unknown pre-grasp position");
      return;
      break;
  }
  updateControls();
}

void Object::createMesh()
{
  mesh_.pose.position.z = -0.5 * scale_.z;

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
  scale_.x = bounding_box_lwh_.x;
  scale_.y = bounding_box_lwh_.y;
  scale_.z = bounding_box_lwh_.z;

  setPose(pose);
}

void Object::create()
{
  clearObject();

  scale_.x = bounding_box_lwh_.x;
  scale_.y = bounding_box_lwh_.y;
  scale_.z = bounding_box_lwh_.z;

  object_.header.frame_id = frame_id_;
  object_.name = name_;
  //object_.description = description_;
  object_.pose = pose_;
  object_.scale = maxScale(scale_);

  createMesh();

  control_.name = "object_control";
  control_.interaction_mode = InteractiveMarkerControl::BUTTON;
  control_.always_visible = true;
  control_.markers.push_back(mesh_);

  object_.controls.clear();
  object_.controls.push_back(control_);

  if (scale_.x != 0.0f && scale_.y != 0.0f && scale_.z != 0.0f)
    BoundingBox::createBoundingBoxControl(0.0f, 0.0f, 0.0f);

  createMenu();
}

void Object::insert()
{
  create();
  updateControls();
  server_->insert(object_, boost::bind(&Object::objectCallback, this, _1));
  menu_handler_.apply(*server_, name_);
}

}
