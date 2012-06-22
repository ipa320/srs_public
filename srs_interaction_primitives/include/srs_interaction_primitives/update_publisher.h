/******************************************************************************
 * \file
 *
 * $Id: UpdatePublisher.h 676 2012-04-19 18:32:07Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 21/02/2012
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
#ifndef UPDATEPUBLISHER_H_
#define UPDATEPUBLISHER_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include "topics_list.h"

#define BUFFER_SIZE 5

namespace srs_interaction_primitives
{

/**
 * @brief Type of Primitive's update.
 */
enum UpdateType
{
  UPDATE_POSE, UPDATE_SCALE, MENU_CLICKED, MOVEMENT_CHANGED, TAG_CHANGED, MOVE_ARM_TO_PREGRASP
};

/**
 * This class publishes updates of BUT GUI Primitives.
 *
 * @author Tomas Lokaj
 */
class UpdatePublisher
{
public:
  /**
   * @brief Constructor
   * @param name is Interactive Marker's name
   */
  UpdatePublisher(std::string im_name, int im_type);
  UpdatePublisher()
  {
  }

  /**
   * @brief Destructor
   */
  virtual ~UpdatePublisher()
  {
  }

  /**
   * @brief Gets update topic
   * @param update type is update's type
   */
  std::string getUpdateTopic(int update_type);

  /**
   * @brief Publishes scale changed message
   * @param new_scale is new scale value
   * @param scale_change is scale value change
   */
  void publishScaleChanged(geometry_msgs::Vector3 new_scale, geometry_msgs::Vector3 scale_change);

  /**
   * @brief Publishes pose changed message
   * @param new_pose is new pose value
   * @param pose_change is pose value change
   */

  void publishPoseChanged(geometry_msgs::Pose new_pose, geometry_msgs::Pose pose_change);
  /**
   * @brief Publishes menu clicked message
   * @param title is menu entry title
   * @param state is menu entry state
   */

  void publishMenuClicked(std::string title, interactive_markers::MenuHandler::CheckState state);

  /**
   * @brief Publishes movement changed message
   * @param new_direction is new direction value
   * @param direction_change is direction value change
   * @param new_velocity is new velocity value
   * @param velocity_change is velocity value change
   */
  void publishMovementChanged(geometry_msgs::Quaternion new_direction, geometry_msgs::Quaternion direction_change,
                              float new_velocity, float velocity_change);

  /**
   * @brief Publishes movement changed message
   * @param new_tag is new tag value
   */
  void publishTagChanged(std::string new_tag);

  /**
   * @brief Publishes pre-grasp position' id to which arm has to move
   * @param pos_id is pre-grasp position's id
   */
  void publishMoveArmToPreGrasp(int pos_id);

private:
  // Interactive Marker name
  std::string im_name_;
  // Interactive Marker type
  int im_type_;
  // Node handler
  ros::NodeHandle nh_;
  // Publishers
  ros::Publisher scaleChangedPublisher_, poseChangedPublisher_, menuClickedPublisher_, movementChangedPublisher_,
                 tagChangedPublisher_, moveArmToPreGraspPublisher_;

};

}

#endif /* UPDATEPUBLISHER_H_ */
