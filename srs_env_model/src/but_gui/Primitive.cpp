/******************************************************************************
 * \file
 *
 * $Id: Primitive.cpp 603 2012-04-16 10:50:03Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 24/11/2011
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

#include <but_gui/Primitive.h>

namespace but_gui
{

Primitive::Primitive(InteractiveMarkerServerPtr server, string frame_id, string name, int type)
{
  server_ = server;
  frame_id_ = frame_id;
  name_ = name;
  primitive_type_ = type;
  menu_created_ = false;

  show_movement_control_ = show_scale_control_ = show_rotation_control_ = show_measure_control_
      = show_description_control_ = show_pregrasp_control_ = show_trajectory_control_ = false;

  updatePublisher_ = new UpdatePublisher(name_, primitive_type_);

  color_green_a02_.r = 0.0;
  color_green_a02_.g = 1.0;
  color_green_a02_.b = 0.0;
  color_green_a02_.a = 0.2;
}

void Primitive::defaultCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  InteractiveMarker object;
  if (server_->get(name_, object))
  {
    if (feedback->event_type == InteractiveMarkerFeedback::MOUSE_UP)
    {
      Pose pose_change;
      pose_change.position.x = pose_.position.x - object.pose.position.x;
      pose_change.position.y = pose_.position.y - object.pose.position.y;
      pose_change.position.z = pose_.position.z - object.pose.position.z;
      pose_change.orientation.x = pose_.orientation.x - object.pose.orientation.x;
      pose_change.orientation.y = pose_.orientation.y - object.pose.orientation.y;
      pose_change.orientation.z = pose_.orientation.z - object.pose.orientation.z;
      pose_change.orientation.w = pose_.orientation.w - object.pose.orientation.w;

      pose_ = object.pose;
      object_.pose = pose_;

      updatePublisher_->publishPoseChanged(object.pose, pose_change);
    }
  }
  else
    ROS_WARN("Cannot find object in IM Server!");

  // Transfer object into IMS frame
  if (feedback->header.frame_id != frame_id_)
  {
    frame_id_ = feedback->header.frame_id;
    pose_ = object.pose;
    insert();
  }
}

void Primitive::scaleFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  static Vector3 scale_change;

  InteractiveMarker o;
  if (server_->get(name_, o))
  {
    pose_ = o.pose;
    object_.pose = pose_;
  }

  int modificator = 0;
  if (feedback->marker_name == name_ + "_min_x")
  {
    min_size_.x = feedback->pose.position.x;
    modificator = -1;
  }
  else if (feedback->marker_name == name_ + "_max_x")
  {
    max_size_.x = feedback->pose.position.x;
    modificator = 1;
  }
  else if (feedback->marker_name == name_ + "_min_y")
  {
    min_size_.y = feedback->pose.position.y;
    modificator = -1;
  }
  else if (feedback->marker_name == name_ + "_max_y")
  {
    max_size_.y = feedback->pose.position.y;
    modificator = 1;
  }
  else if (feedback->marker_name == name_ + "_min_z")
  {
    min_size_.z = feedback->pose.position.z;
    modificator = -1;
  }
  else if (feedback->marker_name == name_ + "_max_z")
  {
    max_size_.z = feedback->pose.position.z;
    modificator = 1;
  }

  Vector3 ds;
  ds.x = ((max_size_.x - min_size_.x) - scale_.x);
  ds.y = ((max_size_.y - min_size_.y) - scale_.y);
  ds.z = ((max_size_.z - min_size_.z) - scale_.z);

  scale_.x = max_size_.x - min_size_.x;
  scale_.y = max_size_.y - min_size_.y;
  scale_.z = max_size_.z - min_size_.z;
  if (scale_.x < 0.0)
    scale_.x = 0.001;
  else
    pose_.position.x += 0.5 * ds.x * modificator;
  if (scale_.y < 0.0)
    scale_.y = 0.001;
  else
    pose_.position.y += 0.5 * ds.y * modificator;
  if (scale_.z < 0.0)
    scale_.z = 0.001;
  else
    pose_.position.z += 0.5 * ds.z * modificator;

  object_.pose = pose_;
  object_.controls[0].markers[0].scale = scale_;
  object_.scale = but_gui::maxScale(scale_);

  updateControls();

  server_->insert(object_);
  menu_handler_.reApply(*server_);
  server_->applyChanges();

  scale_change.x += ds.x;
  scale_change.y += ds.y;
  scale_change.z += ds.z;

  if (feedback->event_type == InteractiveMarkerFeedback::MOUSE_UP)
  {
    updatePublisher_->publishScaleChanged(scale_, scale_change);
    scale_change.x = 0;
    scale_change.y = 0;
    scale_change.z = 0;
  }
}

InteractiveMarkerControl* Primitive::getControl(string name)
{
  for (unsigned int i = 0; i < object_.controls.size(); i++)
  {
    if (object_.controls[i].name == name)
      return (&object_.controls.at(i));
  }
  return NULL;
}

void Primitive::insert()
{
  server_->insert(object_);
}

void Primitive::erase()
{
  server_->erase(name_);
}



void Primitive::addPreGraspPosition(int pos_id, Vector3 position)
{
  switch (pos_id)
  {
    case GRASP_1:
      pregrasp1_.enabled = true;
      pregrasp1_.position = position;
      break;
    case GRASP_2:
      pregrasp2_.enabled = true;
      pregrasp2_.position = position;
      break;
    case GRASP_3:
      pregrasp3_.enabled = true;
      pregrasp3_.position = position;
      break;
    case GRASP_4:
      pregrasp4_.enabled = true;
      pregrasp4_.position = position;
      break;
    case GRASP_5:
      pregrasp5_.enabled = true;
      pregrasp5_.position = position;
      break;
    case GRASP_6:
      pregrasp6_.enabled = true;
      pregrasp6_.position = position;
      break;
    default:
      ROS_WARN("Unknown pre-grasp position");
      return;
      break;
  }
}

void Primitive::removePreGraspPosition(int pos_id)
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

void Primitive::addMovementControls()
{
  show_movement_control_ = true;

  moveXControl_.name = "move_x";
  moveXControl_.orientation.w = 1;
  moveXControl_.orientation.x = 1;
  moveXControl_.orientation.y = 0;
  moveXControl_.orientation.z = 0;
  moveXControl_.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;

  moveYControl_.name = "move_y";
  moveYControl_.orientation.w = 1;
  moveYControl_.orientation.x = 0;
  moveYControl_.orientation.y = 0;
  moveYControl_.orientation.z = 1;
  moveYControl_.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;

  moveZControl_.name = "move_z";
  moveZControl_.orientation.w = 1;
  moveZControl_.orientation.x = 0;
  moveZControl_.orientation.y = 1;
  moveZControl_.orientation.z = 0;
  moveZControl_.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;

  object_.controls.push_back(moveXControl_);
  object_.controls.push_back(moveYControl_);
  object_.controls.push_back(moveZControl_);
}

void Primitive::addRotationControls()
{
  show_rotation_control_ = true;

  rotateXControl_.name = "rotate_x";
  rotateXControl_.orientation.w = 1;
  rotateXControl_.orientation.x = 1;
  rotateXControl_.orientation.y = 0;
  rotateXControl_.orientation.z = 0;
  rotateXControl_.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;

  rotateYControl.name = "rotate_y";
  rotateYControl.orientation.w = 1;
  rotateYControl.orientation.x = 0;
  rotateYControl.orientation.y = 0;
  rotateYControl.orientation.z = 1;
  rotateYControl.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;

  rotateZControl_.name = "rotate_z";
  rotateZControl_.orientation.w = 1;
  rotateZControl_.orientation.x = 0;
  rotateZControl_.orientation.y = 1;
  rotateZControl_.orientation.z = 0;
  rotateZControl_.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;

  object_.controls.push_back(rotateXControl_);
  object_.controls.push_back(rotateYControl);
  object_.controls.push_back(rotateZControl_);
}

void Primitive::removeMovementControls()
{
  show_movement_control_ = false;
  removeControl("move_x");
  removeControl("move_y");
  removeControl("move_z");
}

void Primitive::removeRotationControls()
{
  show_rotation_control_ = false;
  removeControl("rotate_x");
  removeControl("rotate_y");
  removeControl("rotate_z");
}

void Primitive::removeControl(string name)
{
  for (unsigned int i = 0; i < object_.controls.size(); i++)
  {
    if (object_.controls[i].name == name)
    {
      object_.controls.erase(object_.controls.begin() + i);
      return;
    }
  }
}

void Primitive::addMeasureControl()
{
  show_measure_control_ = true;

  Marker measureMarker;
  measureControl_.markers.clear();
  measureMarker.type = Marker::LINE_LIST;
  measureMarker.color.r = color_.g;
  measureMarker.color.g = color_.b;
  measureMarker.color.b = color_.r;
  measureMarker.color.a = 1.0;
  measureMarker.scale.x = 0.01;
  Point p;
  // Z
  double sx = scale_.x / 2;
  double sy = scale_.y / 2;
  double sz = scale_.z / 2;
  p.x = -sx;
  p.y = -sy;
  p.z = -sz;
  measureMarker.points.push_back(p);
  p.z = sz;
  measureMarker.points.push_back(p);
  // Y
  p.x = -sx;
  p.y = sy;
  p.z = -sz;
  measureMarker.points.push_back(p);
  p.y = -sy;
  measureMarker.points.push_back(p);
  // X
  p.x = sx;
  p.y = -sy;
  p.z = -sz;
  measureMarker.points.push_back(p);
  p.x = -sx;
  measureMarker.points.push_back(p);

  Marker measureText;
  ostringstream text_x, text_y, text_z;
  text_x << scale_.x << "m";
  text_y << scale_.y << "m";
  text_z << scale_.z << "m";
  measureText.type = Marker::TEXT_VIEW_FACING;
  measureText.color = measureMarker.color;
  // Z
  measureText.scale.z = scale_.z * MEASURE_TEXT_SCALE;
  if (measureText.scale.z > MEASURE_TEXT_MAX_SIZE)
    measureText.scale.z = MEASURE_TEXT_MAX_SIZE;
  else if (measureText.scale.z < MEASURE_TEXT_MIN_SIZE)
    measureText.scale.z = MEASURE_TEXT_MIN_SIZE;
  measureText.pose.position.x = -sx - 0.2;
  measureText.pose.position.y = -sy - 0.2;
  measureText.pose.position.z = 0;
  measureText.text = text_z.str();
  measureControl_.markers.push_back(measureText);
  // Y
  measureText.scale.z = scale_.y * MEASURE_TEXT_SCALE;
  if (measureText.scale.z > MEASURE_TEXT_MAX_SIZE)
    measureText.scale.z = MEASURE_TEXT_MAX_SIZE;
  else if (measureText.scale.z < MEASURE_TEXT_MIN_SIZE)
    measureText.scale.z = MEASURE_TEXT_MIN_SIZE;
  measureText.pose.position.x = -sx - 0.2;
  measureText.pose.position.y = 0;
  measureText.pose.position.z = -sz - 0.2;
  measureText.text = text_y.str();
  measureControl_.markers.push_back(measureText);
  // X
  measureText.scale.z = scale_.x * MEASURE_TEXT_SCALE;
  if (measureText.scale.z > MEASURE_TEXT_MAX_SIZE)
    measureText.scale.z = MEASURE_TEXT_MAX_SIZE;
  else if (measureText.scale.z < MEASURE_TEXT_MIN_SIZE)
    measureText.scale.z = MEASURE_TEXT_MIN_SIZE;
  measureText.pose.position.x = 0;
  measureText.pose.position.y = -sy - 0.2;
  measureText.pose.position.z = -sz - 0.2;
  measureText.text = text_x.str();
  measureControl_.markers.push_back(measureText);

  measureControl_.name = "measure_control";
  measureControl_.markers.push_back(measureMarker);
  measureControl_.interaction_mode = InteractiveMarkerControl::NONE;
  measureControl_.always_visible = true;

  object_.controls.push_back(measureControl_);
}

void Primitive::removeMeasureControl()
{
  show_measure_control_ = false;
  removeControl("measure_control");
}

void Primitive::removeDescriptionControl()
{
  show_description_control_ = false;
  removeControl("description_control");
}

void Primitive::addDescriptionControl()
{
  show_description_control_ = true;

  descriptionControl_.markers.clear();

  Marker descriptionMarker;
  descriptionMarker.type = Marker::TEXT_VIEW_FACING;
  descriptionMarker.text = description_;
  descriptionMarker.scale.z = 0.2;
  descriptionMarker.color.r = color_.b;
  descriptionMarker.color.g = color_.r;
  descriptionMarker.color.b = color_.g;
  descriptionMarker.color.a = 1.0;
  descriptionMarker.pose.position.z = object_.scale / 2 + 0.2;

  descriptionControl_.name = "description_control";
  descriptionControl_.markers.push_back(descriptionMarker);
  descriptionControl_.interaction_mode = InteractiveMarkerControl::NONE;
  descriptionControl_.orientation_mode = InteractiveMarkerControl::FIXED;
  descriptionControl_.always_visible = true;

  object_.controls.push_back(descriptionControl_);
}

void Primitive::removeScaleControls()
{
  show_scale_control_ = false;
  server_->erase(name_ + "_max_x");
  server_->erase(name_ + "_max_y");
  server_->erase(name_ + "_max_z");
  server_->erase(name_ + "_min_x");
  server_->erase(name_ + "_min_y");
  server_->erase(name_ + "_min_z");
}

void Primitive::addScaleControls()
{
  show_scale_control_ = true;

  min_size_.x = pose_.position.x - scale_.x * 0.5;
  max_size_.x = pose_.position.x + scale_.x * 0.5;
  min_size_.y = pose_.position.y - scale_.y * 0.5;
  max_size_.y = pose_.position.y + scale_.y * 0.5;
  min_size_.z = pose_.position.z - scale_.z * 0.5;
  max_size_.z = pose_.position.z + scale_.z * 0.5;

  for (int axis = 0; axis < 3; axis++)
  {
    for (int sign = -1; sign <= 1; sign += 2)
    {
      InteractiveMarker int_marker;
      int_marker.header.frame_id = frame_id_;
      int_marker.scale = 1.0;
      int_marker.pose = pose_;

      InteractiveMarkerControl control;
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      control.orientation_mode = InteractiveMarkerControl::INHERIT;
      control.always_visible = false;

      control.orientation.w = 1;

      switch (axis)
      {
        case 0:
          int_marker.name = sign > 0 ? name_ + "_max_x" : name_ + "_min_x";
          int_marker.pose.position.x = sign > 0 ? max_size_.x : min_size_.x;
          control.orientation.x = 1;
          control.orientation.y = 0;
          control.orientation.z = 0;
          break;
        case 1:
          int_marker.name = sign > 0 ? name_ + "_max_y" : name_ + "_min_y";
          int_marker.pose.position.y = sign > 0 ? max_size_.y : min_size_.y;
          control.orientation.x = 0;
          control.orientation.y = 0;
          control.orientation.z = 1;
          break;
        default:
          int_marker.name = sign > 0 ? name_ + "_max_z" : name_ + "_min_z";
          int_marker.pose.position.z = sign > 0 ? max_size_.z : min_size_.z;
          control.orientation.x = 0;
          control.orientation.y = -1;
          control.orientation.z = 0;
          break;
      }

      makeArrow(int_marker, control, 0.5 * sign);

      int_marker.controls.push_back(control);
      server_->insert(int_marker, boost::bind(&Primitive::scaleFeedback, this, _1));
    }
  }
}

void Primitive::updateScaleControls()
{
  min_size_.x = pose_.position.x - scale_.x * 0.5;
  max_size_.x = pose_.position.x + scale_.x * 0.5;
  min_size_.y = pose_.position.y - scale_.y * 0.5;
  max_size_.y = pose_.position.y + scale_.y * 0.5;
  min_size_.z = pose_.position.z - scale_.z * 0.5;
  max_size_.z = pose_.position.z + scale_.z * 0.5;

  for (int axis = 0; axis < 3; axis++)
  {
    for (int sign = -1; sign <= 1; sign += 2)
    {
      string n;
      Pose p = pose_;

      switch (axis)
      {
        case 0:
          n = sign > 0 ? name_ + "_max_x" : name_ + "_min_x";
          p.position.x = sign > 0 ? max_size_.x : min_size_.x;
          break;
        case 1:
          n = sign > 0 ? name_ + "_max_y" : name_ + "_min_y";
          p.position.y = sign > 0 ? max_size_.y : min_size_.y;
          break;
        default:
          n = sign > 0 ? name_ + "_max_z" : name_ + "_min_z";
          p.position.z = sign > 0 ? max_size_.z : min_size_.z;
          break;
      }
      server_->setPose(n, p);
    }
  }
}

void Primitive::addPregraspPositions()
{
  show_pregrasp_control_ = true;
  Point p2, p1;
  Marker arrow;
  arrow.color = color_green_a02_;
  arrow.scale.x = GRASP_ARROW_LENGTH / 4;
  arrow.scale.y = GRASP_ARROW_LENGTH / 2;
  arrow.scale.z = 0.1;
  Marker text;
  text.type = Marker::TEXT_VIEW_FACING;
  text.scale.z = 0.2;
  text.color = color_green_a02_;

  pregrasp1Control_.markers.clear();
  pregrasp2Control_.markers.clear();
  pregrasp3Control_.markers.clear();
  pregrasp4Control_.markers.clear();
  pregrasp5Control_.markers.clear();
  pregrasp6Control_.markers.clear();

  if (pregrasp1_.enabled)
  {
    pregrasp1Control_.name = pregrasp1_.name;
    pregrasp1Control_.interaction_mode = InteractiveMarkerControl::MENU;
    pregrasp1Control_.always_visible = true;
    p1.x = pregrasp1_.position.x;
    p1.y = pregrasp1_.position.y;
    p1.z = pregrasp1_.position.z;
    p2.x = p1.x * GRASP_ARROW_SCALE;
    p2.y = p1.y * GRASP_ARROW_SCALE;
    p2.z = p1.z * GRASP_ARROW_SCALE;
    float d = fabs(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)));
    while ((d < GRASP_ARROW_LENGTH) && (d != 0.0))
    {
      p2.x = p2.x * GRASP_ARROW_SCALE;
      p2.y = p2.y * GRASP_ARROW_SCALE;
      p2.z = p2.z * GRASP_ARROW_SCALE;
      d = fabs(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)));
    }
    arrow.points.clear();
    arrow.points.push_back(p2);
    arrow.points.push_back(p1);
    pregrasp1Control_.markers.push_back(arrow);

    text.pose.position.x = p1.x;
    text.pose.position.y = p1.y;
    text.pose.position.z = p1.z + 0.2;
    text.text = "1";
    pregrasp1Control_.markers.push_back(text);
    object_.controls.push_back(pregrasp1Control_);
  }

  if (pregrasp2_.enabled)
  {
    pregrasp2Control_.name = pregrasp2_.name;
    pregrasp2Control_.interaction_mode = InteractiveMarkerControl::MENU;
    pregrasp2Control_.always_visible = true;
    p1.x = pregrasp2_.position.x;
    p1.y = pregrasp2_.position.y;
    p1.z = pregrasp2_.position.z;
    p2.x = p1.x * GRASP_ARROW_SCALE;
    p2.y = p1.y * GRASP_ARROW_SCALE;
    p2.z = p1.z * GRASP_ARROW_SCALE;
    float d = fabs(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)));
    while ((d < GRASP_ARROW_LENGTH) && (d != 0.0))
    {
      p2.x = p2.x * GRASP_ARROW_SCALE;
      p2.y = p2.y * GRASP_ARROW_SCALE;
      p2.z = p2.z * GRASP_ARROW_SCALE;
      d = fabs(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)));
    }
    arrow.points.clear();
    arrow.points.push_back(p2);
    arrow.points.push_back(p1);
    pregrasp2Control_.markers.push_back(arrow);

    text.pose.position.x = p1.x;
    text.pose.position.y = p1.y;
    text.pose.position.z = p1.z + 0.2;
    text.text = "2";
    pregrasp2Control_.markers.push_back(text);
    object_.controls.push_back(pregrasp2Control_);
  }

  if (pregrasp3_.enabled)
  {
    pregrasp3Control_.name = pregrasp3_.name;
    pregrasp3Control_.interaction_mode = InteractiveMarkerControl::MENU;
    pregrasp3Control_.always_visible = true;
    p1.x = pregrasp3_.position.x;
    p1.y = pregrasp3_.position.y;
    p1.z = pregrasp3_.position.z;
    p2.x = p1.x * GRASP_ARROW_SCALE;
    p2.y = p1.y * GRASP_ARROW_SCALE;
    p2.z = p1.z * GRASP_ARROW_SCALE;
    float d = fabs(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)));
    while ((d < GRASP_ARROW_LENGTH) && (d != 0.0))
    {
      cout << d << endl;
      cout << p2 << endl;
      p2.x = p2.x * GRASP_ARROW_SCALE;
      p2.y = p2.y * GRASP_ARROW_SCALE;
      p2.z = p2.z * GRASP_ARROW_SCALE;
      d = fabs(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)));
    }
    arrow.points.clear();
    arrow.points.push_back(p2);
    arrow.points.push_back(p1);
    pregrasp3Control_.markers.push_back(arrow);

    text.pose.position.x = p1.x;
    text.pose.position.y = p1.y;
    text.pose.position.z = p1.z + 0.2;
    text.text = "3";
    pregrasp3Control_.markers.push_back(text);

    object_.controls.push_back(pregrasp3Control_);
  }

  if (pregrasp4_.enabled)
  {
    pregrasp4Control_.name = pregrasp4_.name;
    pregrasp4Control_.interaction_mode = InteractiveMarkerControl::MENU;
    pregrasp4Control_.always_visible = true;
    p1.x = pregrasp4_.position.x;
    p1.y = pregrasp4_.position.y;
    p1.z = pregrasp4_.position.z;
    p2.x = p1.x * GRASP_ARROW_SCALE;
    p2.y = p1.y * GRASP_ARROW_SCALE;
    p2.z = p1.z * GRASP_ARROW_SCALE;
    float d = fabs(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)));
    while ((d < GRASP_ARROW_LENGTH) && (d != 0.0))
    {
      p2.x = p2.x * GRASP_ARROW_SCALE;
      p2.y = p2.y * GRASP_ARROW_SCALE;
      p2.z = p2.z * GRASP_ARROW_SCALE;
      d = fabs(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)));
    }
    arrow.points.clear();
    arrow.points.push_back(p2);
    arrow.points.push_back(p1);
    pregrasp4Control_.markers.push_back(arrow);

    text.pose.position.x = p1.x;
    text.pose.position.y = p1.y;
    text.pose.position.z = p1.z + 0.2;
    text.text = "4";
    pregrasp4Control_.markers.push_back(text);

    object_.controls.push_back(pregrasp4Control_);
  }

  if (pregrasp5_.enabled)
  {
    pregrasp5Control_.name = pregrasp5_.name;
    pregrasp5Control_.interaction_mode = InteractiveMarkerControl::MENU;
    pregrasp5Control_.always_visible = true;
    p1.x = pregrasp5_.position.x;
    p1.y = pregrasp5_.position.y;
    p1.z = pregrasp5_.position.z;
    p2.x = p1.x * GRASP_ARROW_SCALE;
    p2.y = p1.y * GRASP_ARROW_SCALE;
    p2.z = p1.z * GRASP_ARROW_SCALE;
    float d = fabs(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)));
    while ((d < GRASP_ARROW_LENGTH) && (d != 0.0))
    {
      p2.x = p2.x * GRASP_ARROW_SCALE;
      p2.y = p2.y * GRASP_ARROW_SCALE;
      p2.z = p2.z * GRASP_ARROW_SCALE;
      d = fabs(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)));
    }
    arrow.points.clear();
    arrow.points.push_back(p2);
    arrow.points.push_back(p1);
    pregrasp5Control_.markers.push_back(arrow);

    text.pose.position.x = p1.x;
    text.pose.position.y = p1.y;
    text.pose.position.z = p1.z + 0.2;
    text.text = "5";
    pregrasp5Control_.markers.push_back(text);

    object_.controls.push_back(pregrasp5Control_);
  }

  if (pregrasp6_.enabled)
  {
    pregrasp6Control_.name = pregrasp6_.name;
    pregrasp6Control_.interaction_mode = InteractiveMarkerControl::MENU;
    pregrasp6Control_.always_visible = true;
    p1.x = pregrasp6_.position.x;
    p1.y = pregrasp6_.position.y;
    p1.z = pregrasp6_.position.z;
    p2.x = p1.x * GRASP_ARROW_SCALE;
    p2.y = p1.y * GRASP_ARROW_SCALE;
    p2.z = p1.z * GRASP_ARROW_SCALE;
    float d = fabs(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)));
    while ((d < GRASP_ARROW_LENGTH) && (d != 0.0))
    {
      p2.x = p2.x * GRASP_ARROW_SCALE;
      p2.y = p2.y * GRASP_ARROW_SCALE;
      p2.z = p2.z * GRASP_ARROW_SCALE;
      d = fabs(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)));
    }
    arrow.points.clear();
    arrow.points.push_back(p2);
    arrow.points.push_back(p1);
    pregrasp6Control_.markers.push_back(arrow);

    text.pose.position.x = p1.x;
    text.pose.position.y = p1.y;
    text.pose.position.z = p1.z + 0.2;
    text.text = "6";
    pregrasp6Control_.markers.push_back(text);

    object_.controls.push_back(pregrasp6Control_);
  }

}

void Primitive::removePregraspPositions()
{
  show_pregrasp_control_ = false;
  removeControl(pregrasp1_.name);
  removeControl(pregrasp2_.name);
  removeControl(pregrasp3_.name);
  removeControl(pregrasp4_.name);
  removeControl(pregrasp5_.name);
  removeControl(pregrasp6_.name);
}

void Primitive::removeTrajectoryControls()
{
  show_trajectory_control_ = false;
  removeControl("trajectory_control");
}

void Primitive::addTrajectoryControls()
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
  velocityText << velocity_ << "m/s";
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

void Primitive::updateControls()
{
  if (show_measure_control_)
  {
    removeMeasureControl();
    addMeasureControl();
  }
  if (show_rotation_control_)
  {
    removeRotationControls();
    addRotationControls();
  }
  if (show_movement_control_)
  {
    removeMovementControls();
    addMovementControls();
  }
  if (show_description_control_)
  {
    removeDescriptionControl();
    addDescriptionControl();
  }
  if (show_scale_control_)
  {
    updateScaleControls();
  }
  if (show_pregrasp_control_)
  {
    removePregraspPositions();
    addPregraspPositions();
  }
  if (show_trajectory_control_)
  {
    removeTrajectoryControls();
    addTrajectoryControls();
  }
}

void Primitive::clearObject()
{
  control_.markers.clear();
  object_.controls.clear();
  updateControls();
}

void Primitive::changeColor(ColorRGBA color)
{
  color_ = color;
  control_.markers.clear();
  object_.controls.clear();
  insert();
}

float maxScale(Vector3 scale)
{
  if (scale.x > scale.y)
  {
    if (scale.x > scale.z)
    {
      return scale.x;
    }
    else
    {
      return scale.z;
    }
  }
  else
  {
    if (scale.y > scale.z)
    {
      return scale.y;
    }
    else
    {
      return scale.z;
    }
  }
}

float minScale(Vector3 scale)
{
  if (scale.x < scale.y)
  {
    if (scale.x < scale.z)
    {
      return scale.x;
    }
    else
    {
      return scale.z;
    }
  }
  else
  {
    if (scale.y < scale.z)
    {
      return scale.y;
    }
    else
    {
      return scale.z;
    }
  }
}

}
