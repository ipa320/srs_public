/******************************************************************************
 * \file
 *
 * $Id: Plane.h 676 2012-04-19 18:32:07Z xlokaj03 $
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
#ifndef PLANE_H_
#define PLANE_H_

#include "primitive.h"

namespace srs_interaction_primitives
{
/**
 * This class represents a Plane primitive.
 *
 * Plane shows only a simple plane which can be tagged as table desk, wall, etc.

 * @author Tomas Lokaj
 * @see http://ros.org/wiki/srs_env_model#Plane
 */
class Plane : public Primitive
{
public:
  /**
   * Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name_ is name of this plane
   */
  Plane(InteractiveMarkerServerPtr server, std::string frame_id, std::string name);

  /**
   * Insert this Plane into Interactive Marker Server
   */
  void insert();

  /**
   * Callback for menu
   */
  void menuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

protected:
  visualization_msgs::Marker mesh_;
  std::string tag_;


  void create();
  void createMenu();
};

}

#endif /* PLANE_H_ */
