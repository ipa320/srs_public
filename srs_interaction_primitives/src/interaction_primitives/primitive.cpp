/******************************************************************************
 * \file
 *
 * $Id: Primitive.cpp 676 2012-04-19 18:32:07Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
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

#include <srs_interaction_primitives/primitive.h>

using namespace std;
using namespace interactive_markers;
using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std_msgs;

namespace srs_interaction_primitives
{

Primitive::Primitive(InteractiveMarkerServerPtr server, string frame_id, string name, int type)
{
  server_ = server;
  frame_id_ = frame_id;
  name_ = name;
  primitive_type_ = type;
  menu_created_ = false;
  pose_type_ = PoseType::POSE_CENTER;

  show_movement_control_ = show_scale_control_ = show_rotation_control_ = show_measure_control_ =
      show_description_control_ = show_trajectory_control_ = false;

  updatePublisher_ = new UpdatePublisher(name_, primitive_type_);
  color_green_a01_.r = 0.0;
  color_green_a01_.g = 1.0;
  color_green_a01_.b = 0.0;
  color_green_a01_.a = 0.1;

  scale_saved_ = 0;
}

void Primitive::defaultCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
	// Update the scale marker positions
	if( scale_saved_ > 1 )
	{
		Vector3 scale_change;
		scale_change.x = scale_.x - scale_prev_.x;
		scale_change.y = scale_.y - scale_prev_.y;
		scale_change.z = scale_.z - scale_prev_.z;
		scale_saved_ = 0;

		updatePublisher_->publishScaleChanged(scale_, scale_change);

		removeScaleControls();
		addScaleControls();

		server_->applyChanges();
	}

	// Process the feedback
	if (feedback->event_type == InteractiveMarkerFeedback::MOUSE_UP)
	{
		geometry_msgs::Pose npose = feedback->pose;
		if (frame_id_ != feedback->header.frame_id)
		{
			//ROS_INFO("Different frame ids");
			try
			{
				tfListener = new tf::TransformListener();
				tfListener->waitForTransform(feedback->header.frame_id, frame_id_, feedback->header.stamp, ros::Duration(2.0));
				tfListener->lookupTransform(feedback->header.frame_id, frame_id_, feedback->header.stamp, feedbackToDefaultTransform);
				delete tfListener;
				transformer.setTransform(feedbackToDefaultTransform);

				btVector3 position;
				position.setX(feedback->pose.position.x);
				position.setY(feedback->pose.position.y);
				position.setZ(feedback->pose.position.z);

				btQuaternion orientation;
				orientation.setX(feedback->pose.orientation.x);
				orientation.setY(feedback->pose.orientation.y);
				orientation.setZ(feedback->pose.orientation.z);
				orientation.setW(feedback->pose.orientation.w);

				tf::Stamped<btTransform> pose;
				pose.setOrigin(position);
				pose.setRotation(orientation);
				pose.frame_id_ = feedback->header.frame_id;
				transformer.transformPose(frame_id_, pose, pose);

				npose.position.x = pose.getOrigin().getX();
				npose.position.y = pose.getOrigin().getY();
				npose.position.z = pose.getOrigin().getZ();
				npose.orientation.x = pose.getRotation().getX();
				npose.orientation.y = pose.getRotation().getY();
				npose.orientation.z = pose.getRotation().getZ();
				npose.orientation.w = pose.getRotation().getW();
			}
			catch (tf::TransformException& ex)
			{
				ROS_WARN("Transform error!");
				delete tfListener;
				return;
			}
		}

		// Calculate the pose change in the frame_id_
		pose_change.position = npose.position;
		pose_change.position.x -= pose_.position.x;
		pose_change.position.y -= pose_.position.y;
		pose_change.position.z -= pose_.position.z;

		btQuaternion orientation(pose_.orientation.x,
								 pose_.orientation.y,
								 pose_.orientation.z,
								 pose_.orientation.w);
		orientation.inverse();
		orientation *= btQuaternion(npose.orientation.x,
				                    npose.orientation.y,
								    npose.orientation.z,
								    npose.orientation.w);

		pose_change.orientation.x = orientation.getX();
		pose_change.orientation.y = orientation.getY();
		pose_change.orientation.z = orientation.getZ();
		pose_change.orientation.w = orientation.getW();

		// Store the new position
		pose_ = npose;

		insert();
		server_->applyChanges();

		// Majkl: Why the pose changed was published in the message frame ID and not in the IM's one?
		//    updatePublisher_->publishPoseChanged(feedback->pose, pose_change);
		updatePublisher_->publishPoseChanged(pose_, pose_change);
	}
}

void Primitive::scaleFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	btVector3 center(pose_.position.x, pose_.position.y, pose_.position.z);
	btQuaternion orient(pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);

	if (feedback->event_type == InteractiveMarkerFeedback::MOUSE_DOWN)
	{
		scale_prev_.x = scale_.x;
		scale_prev_.y = scale_.y;
		scale_prev_.z = scale_.z;
		scale_saved_ = 1;
	}
	else if (feedback->event_type == InteractiveMarkerFeedback::MOUSE_UP && scale_saved_ > 0)
	{
		Vector3 scale_change;
		scale_change.x = scale_.x - scale_prev_.x;
		scale_change.y = scale_.y - scale_prev_.y;
		scale_change.z = scale_.z - scale_prev_.z;
		scale_saved_ = 0;

		updatePublisher_->publishScaleChanged(scale_, scale_change);

		removeScaleControls();
		addScaleControls();

		server_->applyChanges();
	}
	else if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE  && scale_saved_ > 0)
	{
		++scale_saved_;

		geometry_msgs::Pose npose = feedback->pose;
		if( frame_id_ != feedback->header.frame_id )
		{
//		  ROS_INFO("different frame");
		  try
		  {
			tfListener = new tf::TransformListener();
			tfListener->waitForTransform(feedback->header.frame_id, frame_id_, feedback->header.stamp, ros::Duration(2.0));
			tfListener->lookupTransform(feedback->header.frame_id, frame_id_, feedback->header.stamp, feedbackToDefaultTransform);
			delete tfListener;
			transformer.setTransform(feedbackToDefaultTransform);

			btVector3 position;
			position.setX(feedback->pose.position.x);
			position.setY(feedback->pose.position.y);
			position.setZ(feedback->pose.position.z);

			btQuaternion orientation;
			orientation.setX(feedback->pose.orientation.x);
			orientation.setY(feedback->pose.orientation.y);
			orientation.setZ(feedback->pose.orientation.z);
			orientation.setW(feedback->pose.orientation.w);

			tf::Stamped<btTransform> pose;
			pose.setOrigin(position);
			pose.setRotation(orientation);
			pose.frame_id_ = feedback->header.frame_id;
			transformer.transformPose(frame_id_, pose, pose);

			npose.position.x = pose.getOrigin().getX();
			npose.position.y = pose.getOrigin().getY();
			npose.position.z = pose.getOrigin().getZ();
			npose.orientation.x = pose.getRotation().getX();
			npose.orientation.y = pose.getRotation().getY();
			npose.orientation.z = pose.getRotation().getZ();
			npose.orientation.w = pose.getRotation().getW();
		  }
		  catch (tf::TransformException& ex)
		  {
			ROS_WARN("Transform error!");
			delete tfListener;
			return;
		  }
		}

		btVector3 position(npose.position.x, npose.position.y, npose.position.z);
		btVector3 vec = position - center;
		double size = vec.length();

		btVector3 rotated = quatRotate(orient, btVector3(1, 0, 0));
		int xsign = 1;
		if( rotated.x() < 0 )
		{
			xsign = -1;
		}
		rotated = quatRotate(orient, btVector3(0, 1, 0));
		int ysign = 1;
		if( rotated.y() < 0 )
		{
			ysign = -1;
		}
		rotated = quatRotate(orient, btVector3(0, 0, 1));
		int zsign = 1;
		if( rotated.z() < 0 )
		{
			zsign = -1;
		}

		if (feedback->marker_name == name_ + "_min_x")
		{
			int sign = (pose_.position.x - npose.position.x) > 0.0 ? 1 : -1;
			scale_.x = scale_prev_.x + xsign * sign * size;
//			scale_.x = scale_prev_.x + xsign * (pose_.position.x - npose.position.x);
		}
		else if (feedback->marker_name == name_ + "_max_x")
		{
			int sign = (npose.position.x - pose_.position.x) > 0.0 ? 1 : -1;
			scale_.x = scale_prev_.x + xsign * sign * size;
//			scale_.x = scale_prev_.x + xsign * (npose.position.x - pose_.position.x);
		}
		else if (feedback->marker_name == name_ + "_min_y")
		{
			int sign = (pose_.position.y - npose.position.y) > 0.0 ? 1 : -1;
			scale_.y = scale_prev_.y + ysign * sign * size;
//			scale_.y = scale_prev_.y + ysign * (pose_.position.y - npose.position.y);
		}
		else if (feedback->marker_name == name_ + "_max_y")
		{
			int sign = (npose.position.y - pose_.position.y) > 0.0 ? 1 : -1;
			scale_.y = scale_prev_.y + ysign * sign * size;
//			scale_.y = scale_prev_.y + ysign * (npose.position.y - pose_.position.y);
		}
		else if (feedback->marker_name == name_ + "_min_z")
		{
			int sign = (pose_.position.z - npose.position.z) > 0.0 ? 1 : -1;
			scale_.z = scale_prev_.z + zsign * sign * size;
//			scale_.z = scale_prev_.z + zsign * (pose_.position.z - npose.position.z);
		}
		else if (feedback->marker_name == name_ + "_max_z")
		{
			int sign = (npose.position.z - pose_.position.z) > 0.0 ? 1 : -1;
			scale_.z = scale_prev_.z + zsign * sign * size;
//			scale_.z = scale_prev_.z + zsign * (npose.position.z - pose_.position.z);
		}

		if (scale_.x < 0.0) scale_.x = 0.001;
		if (scale_.y < 0.0) scale_.y = 0.001;
		if (scale_.z < 0.0) scale_.z = 0.001;

		if ((object_.controls.size() > 0) && (object_.controls[0].markers.size() > 0))
		{
			object_.controls[0].markers[0].scale = scale_;
		}
		object_.scale = SCALE_COEFF * srs_interaction_primitives::maxScale(scale_);

		updateControls();

		insert();
		server_->applyChanges();
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

  // Scale controls are managed as separate interactive markers...
  removeScaleControls();
}

void Primitive::addMovementControls()
{
  show_movement_control_ = true;

  moveXControl_.name = "move_x";
//  moveXControl_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  moveXControl_.orientation.w = 1;
  moveXControl_.orientation.x = 1;
  moveXControl_.orientation.y = 0;
  moveXControl_.orientation.z = 0;
  moveXControl_.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;

  moveYControl_.name = "move_y";
//  moveYControl_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0.5 * M_PI);
  moveYControl_.orientation.w = 1;
  moveYControl_.orientation.x = 0;
  moveYControl_.orientation.y = 0;
  moveYControl_.orientation.z = 1;
  moveYControl_.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;

  moveZControl_.name = "move_z";
//  moveZControl_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -0.5 * M_PI, 0);
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
//  rotateXControl_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  rotateXControl_.orientation.w = 1;
  rotateXControl_.orientation.x = 1;
  rotateXControl_.orientation.y = 0;
  rotateXControl_.orientation.z = 0;
  rotateXControl_.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;

  rotateYControl.name = "rotate_y";
//  rotateYControl.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0.5 * M_PI);
  rotateYControl.orientation.w = 1;
  rotateYControl.orientation.x = 0;
  rotateYControl.orientation.y = 0;
  rotateYControl.orientation.z = 1;
  rotateYControl.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;

  rotateZControl_.name = "rotate_z";
//  moveZControl_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -0.5 * M_PI, 0);
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
  ROS_WARN("REMOVING %s", name.c_str());
  for (unsigned int i = 0; i < object_.controls.size(); i++)
  {
    std::cout << object_.controls[i].name << std::endl;
    if (object_.controls[i].name == name)
    {
      ROS_WARN("OK");
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
  text_x << setprecision(2) << fixed << scale_.x << "m";
  text_y << setprecision(2) << fixed << scale_.y << "m";
  text_z << setprecision(2) << fixed << scale_.z << "m";
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
//  descriptionMarker.scale.z = 0.2;
//  descriptionMarker.color.r = color_.b;
//  descriptionMarker.color.g = color_.r;
//  descriptionMarker.color.b = color_.g;
//  descriptionMarker.pose.position.z = object_.scale / 2 + 0.2;
  descriptionMarker.scale.z = object_.scale * 0.2;
  descriptionMarker.color.r = 1.0;
  descriptionMarker.color.g = 1.0;
  descriptionMarker.color.b = 1.0;
  descriptionMarker.color.a = 1.0;
  descriptionMarker.pose.position.z = object_.scale * 0.8 + 0.2;

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
//      int_marker.scale = 1.0;
      int_marker.scale = 0.6 * srs_interaction_primitives::maxScale(scale_);
      int_marker.pose = pose_;

      InteractiveMarkerControl control;
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      control.orientation_mode = InteractiveMarkerControl::INHERIT;
      control.always_visible = false;

      switch (axis)
      {
        case 0:
          int_marker.name = sign > 0 ? name_ + "_max_x" : name_ + "_min_x";
//          control.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
          control.orientation.w = 1;
          control.orientation.x = 1;
          control.orientation.y = 0;
          control.orientation.z = 0;
          break;
        case 1:
          int_marker.name = sign > 0 ? name_ + "_max_y" : name_ + "_min_y";
//          control.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0.5 * M_PI);
          control.orientation.w = 1;
          control.orientation.x = 0;
          control.orientation.y = 0;
          control.orientation.z = 1;
          break;
        default:
//          int_marker.name = sign > 0 ? name_ + "_max_z" : name_ + "_min_z";
          int_marker.name = sign < 0 ? name_ + "_max_z" : name_ + "_min_z";
//          control.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -0.5 * M_PI, 0);
          control.orientation.w = 1;
          control.orientation.x = 0;
          control.orientation.y = 1;
          control.orientation.z = 0;
          break;
      }

//      makeArrow(int_marker, control, 3.0 * sign);
      makeArrow(int_marker, control, 3.5 * sign);

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
      switch (axis)
      {
        case 0:
          n = sign > 0 ? name_ + "_max_x" : name_ + "_min_x";
          break;
        case 1:
          n = sign > 0 ? name_ + "_max_y" : name_ + "_min_y";
          break;
        default:
          n = sign > 0 ? name_ + "_max_z" : name_ + "_min_z";
          break;
      }
      server_->setPose(n, pose_);
      server_->applyChanges();
    }
  }
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
