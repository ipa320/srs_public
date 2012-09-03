/******************************************************************************
 * \file
 *
 * $Id: Object.h 676 2012-04-19 18:32:07Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd.mm.2011
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
#ifndef OBJECT_H_
#define OBJECT_H_

#include "bounding_box.h"

namespace srs_interaction_primitives
{
/**
 * This class represents detected object and it's bounding box.
 *
 * Object with Bounding Box is designed to be the direct output of the detectors.
 * It shows object's mesh together with it's Bounding Box.
 * It can also show pre-grasp positions and can be translated, rotated or scaled.
 *
 * @author Tomas Lokaj
 * @see http://ros.org/wiki/srs_env_model#Object
 */
class Object : public BoundingBox
{
public:
  /**
   * Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this object
   */
  Object(InteractiveMarkerServerPtr server, std::string frame_id, std::string name);

  /**
   * Creates and inserts object into Interactive Marker Server
   */
  void insert();

  /**
   * Sets object's bounding box
   * @param bounding_box_lwh is bounding box length, width and height
   */
  void setBoundingBoxLWH(geometry_msgs::Point bounding_box_lwh)
  {
    bounding_box_lwh_ = bounding_box_lwh;
  }

  /**
   * Gets object's bounding box
   * @return bounding_box_lwh is bounding box length, width and height
   */
  geometry_msgs::Point getBoundingBoxLWH()
  {
    return bounding_box_lwh_;
  }

  /**
   * Sets position orientation of the object and dimension of the bounding box.
   * @param pose is object's position and orientation
   * @param bounding_box_lwh is length, width and height of the bounding box
   */
  void setPoseLWH(geometry_msgs::Pose pose, geometry_msgs::Point bounding_box_lwh);

  /**
   * Callback
   */
  void objectCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
   * Menu callback
   */
  void menuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

private:
  void create();
  void createMenu();
  void createMesh();
  void createBoundingBoxControl();

  geometry_msgs::Point bounding_box_lwh_;
  bool translated_;
};

}

#endif /* OBJECT_H_ */

