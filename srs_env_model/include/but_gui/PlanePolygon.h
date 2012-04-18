/******************************************************************************
 * \file
 *
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 02/04/2012
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

#ifndef PLANEPOLYGON_H_
#define PLANEPOLYGON_H_

#include "Plane.h"
#include <Eigen/Geometry>

#define LINE_WIDTH 0.01

namespace but_gui
{
/**
 * This class represents a Plane Polygon primitive.
 *
 * It's a variation of the Plane, which is defined with planar polygon.
 * It's shown as transparent plane and polygon inside it.
 *
 * @author Tomas Lokaj
 * @see http://ros.org/wiki/srs_env_model#PlanePolygon
 */
class PlanePolygon : public Plane
{
public:
  /**
   * @brief Constructor.
   *
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this object
   */
  PlanePolygon(InteractiveMarkerServerPtr server, string frame_id, string name);

protected:
  void create();

private:
  Marker polygon_mesh_;
};

}

#endif /* PLANEPOLYGON_H_ */
