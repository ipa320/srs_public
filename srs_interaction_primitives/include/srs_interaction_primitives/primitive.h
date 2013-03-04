/******************************************************************************
 * \file
 *
 * $Id: Primitive.h 676 2012-04-19 18:32:07Z xlokaj03 $
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

#pragma once
#ifndef PRIMITIVE_H_
#define PRIMITIVE_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/tools.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/ColorRGBA.h>
#include <OGRE/OgreVector3.h>
#include <arm_navigation_msgs/Shape.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <srs_interaction_primitives/parameters_list.h>
#include <srs_interaction_primitives/PrimitiveType.h>
#include <srs_interaction_primitives/PoseType.h>
#include "update_publisher.h"

#define MEASURE_TEXT_SCALE 0.2
#define MEASURE_TEXT_MAX_SIZE 0.3
#define MEASURE_TEXT_MIN_SIZE 0.1

#define GRASP_ARROW_LENGTH 0.25
#define GRASP_ARROW_WIDTH 0.2
#define GRASP_TEXT_OFFSET 0.1
#define GRASP_TEXT_SIZE 0.1
#define GRASP_POINT_SCALE 0.05
#define GRASP_TRANSPARENCY 0.3

namespace srs_interaction_primitives
{

typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;
typedef geometry_msgs::Vector3 Scale;

/**
 * @brief Returns maximal scale value
 * @param scale is vector of scale values
 */
float maxScale(geometry_msgs::Vector3 scale);

/**
 * @brief Returns minimal scale value
 * @param scale is vector of scale values
 */
float minScale(geometry_msgs::Vector3 scale);

const float SCALE_COEFF = 1.0;

/**
 * This class is parent class for BUT GUI primitives
 *
 * @author Tomas Lokaj
 */
class Primitive
{
public:
  /**
   * @brief Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this primitive
   */
  Primitive(InteractiveMarkerServerPtr server, std::string frame_id, std::string name, int type);

  /**
   * @brief Constructor
   */
  Primitive()
  {
  }

  /**
   * @brief Destructor
   */
  virtual ~Primitive()
  {
    this->erase();
    server_->applyChanges();
  }

  /**
   * @brief Insert primitive into Interactive Marker Server
   */
  virtual void insert();

  /**
   * @brief Erase primitive from Interactive Marker Server
   */
  virtual void erase();

  /**
   * @brief Change primitive's color
   * @param color is primitives's color
   */
  void changeColor(std_msgs::ColorRGBA color);

  /**
   * @brief Sets the way to set the pose
   * @param type of the pose
   */
  void setPoseType(int type)
  {
    if (type == PoseType::POSE_CENTER || type == PoseType::POSE_BASE)
    {
      pose_type_ = type;
    }
  }

  /**
   * @brief Gets the way to set the pose
   * @return type of the pose
   */
  int getPoseType()
  {
    return pose_type_;
  }

  /**
   * @brief Sets type of this primitive.
   * @param type is primitive's type
   */
  void setPrimitiveType(int type)
  {
    primitive_type_ = type;
    delete updatePublisher_;
    updatePublisher_ = new UpdatePublisher(name_, primitive_type_);
  }

  /**
   * @brief Gets type of this primitive.
   * @return primitives type
   */
  int getPrimitiveType()
  {
    return primitive_type_;
  }

  /**
   * @brief Gets update topic
   * @param update_type is update's type
   */
  std::string getUpdateTopic(int update_type)
  {
    return updatePublisher_->getUpdateTopic(update_type);
  }

  /**
   * @brief Gets primitive's name
   * @return primitive's name
   */
  std::string getName()
  {
    return name_;
  }

  /**
   * @brief Sets color of the primitive
   * @param color is primitive's color
   */
  void setColor(std_msgs::ColorRGBA color)
  {
    color_ = color;
  }

  /**
   * @brief Gets primitive's color
   * @return primitive's color
   */
  std_msgs::ColorRGBA getColor()
  {
    return color_;
  }

  /**
   * @brief Sets position and orientation to object
   * @param pose is primitive's position and orientation
   */
  void setPose(geometry_msgs::Pose pose)
  {
    if (pose_type_ == PoseType::POSE_BASE)
    {
      pose.position.z += scale_.z * 0.5;
    }

    geometry_msgs::Pose pose_change;
    pose_change.position.x = pose.position.x - pose_.position.x;
    pose_change.position.y = pose.position.y - pose_.position.y;
    pose_change.position.z = pose.position.z - pose_.position.z;
    pose_change.orientation.x = pose.orientation.x - pose_.orientation.x;
    pose_change.orientation.y = pose.orientation.y - pose_.orientation.y;
    pose_change.orientation.z = pose.orientation.z - pose_.orientation.z;
    pose_change.orientation.w = pose.orientation.w - pose_.orientation.w;
    pose_ = pose;
    object_.pose = pose_;
    updatePublisher_->publishPoseChanged(pose_, pose_change);
  }

  /**
   * @brief Gets primitive's position and orientation
   * @return primitive's position and orientation
   */
  geometry_msgs::Pose getPose()
  {
    return pose_;
  }

  /**
   * @brief Sets scale to object
   * @param scale is primitive's scale
   */
  void setScale(geometry_msgs::Vector3 scale)
  {
    if (pose_type_ == PoseType::POSE_BASE)
    {
      pose_.position.z -= scale_.z * 0.5;
      pose_.position.z += scale.z * 0.5;
    }

    geometry_msgs::Vector3 scale_change;
    scale_change.x = scale.x - scale_.x;
    scale_change.y = scale.y - scale_.y;
    scale_change.z = scale.z - scale_.z;
    scale_ = scale;
    object_.scale = SCALE_COEFF * srs_interaction_primitives::maxScale(scale_);
    updatePublisher_->publishScaleChanged(scale_, scale_change);
  }

  /**
   * @brief Gets primitive's scale
   * @return primitive's scale
   */
  geometry_msgs::Vector3 getScale()
  {
    return scale_;
  }

  /**
   * @brief Sets description to object
   * @param description is primitive's description
   */
  void setDescription(std::string description)
  {
    description_ = description;
  }

  /**
   * Gets primitive's description
   * @return primitive's description
   */
  std::string getDescription()
  {
    return description_;
  }

  /**
   * @brief Sets fixed frame to object
   * @param frame_id is primitive's fixed frame
   */
  void setFrameID(std::string frame_id)
  {
    frame_id_ = frame_id;
  }

  /**
   * @brief Gets primitive's fixed frame
   * @return primitive's fixed frame
   */
  std::string getFrameID()
  {
    return frame_id_;
  }

  /**
   * @brief Updates visible controls
   */
  void updateControls();

  /**
   * @brief Returns control
   * @param name is controls name
   */
  visualization_msgs::InteractiveMarkerControl * getControl(std::string name);

  /**
   * @brief Default callback for objects
   */
  virtual void defaultCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

protected:
  /**
   * @brief Create Object
   */
  virtual void create()
  {
  }

  /**
   * @brief Create menu
   */
  virtual void createMenu()
  {
  }

  /**
   * @brief Adds description controls
   */
  void addDescriptionControl();

  /**
   * @brief Adds measure controls
   */
  void addMeasureControl();

  /**
   * @brief Adds movement controls
   * @param marker is Interactive marker
   */
  void addMovementControls();

  /**
   * @brief Adds rotation controls
   */
  void addRotationControls();

  /**
   * @brief Removes movement controls
   */
  void removeMovementControls();

  /**
   * @brief Removes rotation controls
   */
  void removeRotationControls();

  /**
   * @brief Removes measure control
   */
  void removeMeasureControl();

  /**
   * Removes description control
   */
  void removeDescriptionControl();

  /**
   * Removes control with specified name
   * @param name is control's name
   */
  void removeControl(std::string name);

  /**
   * @brief Resets object
   */
  void clearObject();

  /**
   * @brief Adds scale controls
   */
  void addScaleControls();

  /**
   * @brief Updates scale controls
   */
  virtual void updateScaleControls();

  /**
   * @brief Removes scale controls
   */
  void removeScaleControls();

  /**
   * @brief Scale controls feedback
   */
  void scaleFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  geometry_msgs::Vector3 min_size_, max_size_;
  int primitive_type_;
  visualization_msgs::InteractiveMarker object_;
  visualization_msgs::InteractiveMarkerControl control_, descriptionControl_, measureControl_, scaleControl_,
                                               trajectoryControl_;
  visualization_msgs::InteractiveMarkerControl moveXControl_, moveYControl_, moveZControl_, rotateXControl_,
                                               rotateYControl, rotateZControl_;
  interactive_markers::MenuHandler menu_handler_;

// Transform listener
  tf::TransformListener *tfListener;

// Transformer
  tf::Transformer transformer;

// Transformations
  tf::StampedTransform feedbackToDefaultTransform;

// Common attributes
  std::string name_, description_, frame_id_;
  InteractiveMarkerServerPtr server_;
  geometry_msgs::Pose pose_;
  geometry_msgs::Vector3 scale_;
  std_msgs::ColorRGBA color_, color_green_a01_;
  int pose_type_;
  geometry_msgs::Pose pose_change;

  UpdatePublisher *updatePublisher_;

  bool show_movement_control_, show_scale_control_, show_rotation_control_, show_measure_control_,
       show_description_control_,  show_trajectory_control_;
  bool menu_created_;

  geometry_msgs::Vector3 scale_prev_;
  int scale_saved_;
};


}
#endif /* PRIMITIVE_H_ */
