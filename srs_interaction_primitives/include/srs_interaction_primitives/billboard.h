/******************************************************************************
 * \file
 *
 * $Id: Billboard.h 676 2012-04-19 18:32:07Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 27.11.2011
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
#ifndef BILLBOARD_H_
#define BILLBOARD_H_

#include "primitive.h"
#include <srs_interaction_primitives/BillboardType.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMatrix3.h>
#include <OGRE/OgreQuaternion.h>

#define PREDICTIONS_COUNT 3
#define PREDICTION_SPHERE_SIZE 0.1

namespace srs_interaction_primitives
{
/**
 * This class represents Billboard primitive.
 *
 * Billboard represents real world object detected around robot, which is hard to describe with some mesh.
 * Billboard is view facing and iẗ́'s types are specified in srs_env_model/BillboardType message.
 * Billboard can also illustrate the movement of the represented object, e.g., walking person.
 *
 * @author Tomas Lokaj
 * @see http://ros.org/wiki/srs_env_model#Billboard
 */
class Billboard : public Primitive
{
public:
  /**
   * Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this billboard
   */
  Billboard(InteractiveMarkerServerPtr server, std::string frame_id, std::string name);

  /**
   * Sets type to the billboard
   * @param type is billboard's type
   */
  void setType(int type);

  /**
   * Gets billboard's type
   * @return type of the billboard
   */
  int getType();

  /**
   * Inserts this billboard into Interactive marker server
   */
  void insert();

  /**
   * @brief Gets billboard's movement velocity
   * @return billboard's movement velocity
   */
  double getVelocity()
  {
    if (primitive_type_ != srs_interaction_primitives::PrimitiveType::BILLBOARD)
      ROS_WARN("This is object is not a billboard, you cannot get velocity!");
    return velocity_;
  }

  /**
   * @brief Sets direction to the billboard
   * @param direction is billboard's direction
   */
  void setDirection(geometry_msgs::Quaternion direction)
  {
    if (primitive_type_ != srs_interaction_primitives::PrimitiveType::BILLBOARD)
    {
      ROS_WARN("This is object is not a billboard, you cannot set direction!");
      return;
    }

    geometry_msgs::Quaternion direction_change;
    direction_change.x = direction.w - direction_.x;
    direction_change.y = direction.y - direction_.y;
    direction_change.z = direction.z - direction_.z;
    direction_change.w = direction.w - direction_.w;

    updatePublisher_->publishMovementChanged(direction, direction_change, velocity_, 0.0f);

    direction_ = direction;
  }

  /**
   * @brief Gets billboard's movement direction
   * @return billboard's movement direction
   */
  geometry_msgs::Quaternion getDirection()
  {
    if (primitive_type_ != srs_interaction_primitives::PrimitiveType::BILLBOARD)
      ROS_WARN("This is object is not a billboard, you cannot get direction!");
    return direction_;
  }

  /**
   * @brief Sets velocity to the billboard
   * @param velocity is billboard's velocity
   */
  void setVelocity(double velocity)
  {
    if (primitive_type_ != srs_interaction_primitives::PrimitiveType::BILLBOARD)
    {
      ROS_WARN("This is object is not a billboard, you cannot set velocity!");
      return;
    }
    updatePublisher_->publishMovementChanged(direction_, geometry_msgs::Quaternion(), velocity, velocity - velocity_);
    velocity_ = velocity;
  }

  /**
   * @brief Adds trajectory controls
   */
  void addTrajectoryControls();

  /**
   * @brief Removes trajectory control
   */
  void removeTrajectoryControls();

  /**
   * @brief Updates visible controls
   */
  void updateControls();

  /**
   * Callback for menu
   */
  void menuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

private:
  // Billboard's attributes
  double velocity_;
  geometry_msgs::Quaternion direction_;
  int billboard_type_;
  visualization_msgs::Marker mesh_;

  void create();
  void createMenu();
  void createMesh();
  void addTrajectoryPredictionMarkers();
  void removeTrajectoryPredictionMarkers();
};
}
#endif /* BILLBOARD_H_ */
