/******************************************************************************
 * \file
 *
 * $Id: Plane.h 603 2012-04-16 10:50:03Z xlokaj03 $
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

#ifndef PLANE_H_
#define PLANE_H_

#include "Primitive.h"

namespace but_gui
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
  Plane(InteractiveMarkerServerPtr server, string frame_id, string name);

  /**
   * Insert this Plane into Interactive Marker Server
   */
  void insert();

  /**
   * Callback for menu
   */
  void menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback);

protected:
  Marker mesh_;
  string tag_;


  void create();
  void createMenu();
};

}

#endif /* PLANE_H_ */
