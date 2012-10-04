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

#pragma once
#ifndef PLANEPOLYGON_H_
#define PLANEPOLYGON_H_

#include "plane.h"
#include <Eigen/Geometry>

#define LINE_WIDTH 0.01

namespace srs_interaction_primitives
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
  PlanePolygon(InteractiveMarkerServerPtr server, std::string frame_id, std::string name);

  /**
   * @brief Gets plane's normal.
   * @return plane's normal
   */
  Ogre::Vector3 getNormal()
  {
    return normal_;
  }

  /**
   * @brief Sets plane's normal.
   * @param normal is plane's normal
   */
  void setNormal(Ogre::Vector3 normal)
  {
    normal_ = normal;
  }

  /**
   * @brief Sets plane's normal.
   * @param normal is plane's normal
   */
  void setNormal(geometry_msgs::Vector3 normal)
  {
    normal_ = Ogre::Vector3(normal.x, normal.y, normal.z);
  }

  /**
   * @brief Sets PlanePolygon's polygon
   * @param polygon is PlanePolygon's polygon
   */
  void setPolygon(geometry_msgs::Polygon polygon)
  {
    polygon_ = polygon;
  }

protected:
  void create();

private:
  // PlanePolygon's attributes
  Ogre::Vector3 normal_;
  geometry_msgs::Polygon polygon_;
  visualization_msgs::Marker polygon_mesh_;
};

}

#endif /* PLANEPOLYGON_H_ */
