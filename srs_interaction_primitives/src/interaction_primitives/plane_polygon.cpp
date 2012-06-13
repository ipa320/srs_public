/******************************************************************************
 * \file
 *
 * $Id: PlanePolygon.cpp 676 2012-04-19 18:32:07Z xlokaj03 $
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

#include <srs_interaction_primitives/plane_polygon.h>

using namespace std;
using namespace interactive_markers;
using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std_msgs;


namespace srs_interaction_primitives
{

PlanePolygon::PlanePolygon(InteractiveMarkerServerPtr server, string frame_id, string name) :
  Plane(server, frame_id, name)
{
  setPrimitiveType(srs_interaction_primitives::PrimitiveType::PLANE_POLYGON);
}

void PlanePolygon::create()
{
  if (polygon_.points.size() == 0)
  {
    ROS_WARN("Empty polygon!");
    return;
  }

  clearObject();

  Point p, max, min;
  max.x = max.y = max.z = -9999999.0;
  min.x = min.y = min.z = 9999999.0;

  object_.header.frame_id = frame_id_;
  object_.header.stamp = ros::Time::now();
  object_.name = name_;
  object_.description = name_ + " plane";

  polygon_mesh_.type = Marker::LINE_STRIP;
  polygon_mesh_.color = color_;
  polygon_mesh_.color.a = 1;
  polygon_mesh_.header.frame_id = frame_id_;
  polygon_mesh_.scale.x = LINE_WIDTH;

  Point center;
  center.x = 0.0;
  center.y = 0.0;
  center.z = 0.0;

  for (unsigned int i = 0; i < polygon_.points.size(); i++)
  {
    p.x = polygon_.points[i].x;
    p.y = polygon_.points[i].y;
    p.z = polygon_.points[i].z;
    polygon_mesh_.points.push_back(p);
    if (p.x > max.x)
      max.x = p.x;
    else if (p.x < min.x)
      min.x = p.x;
    if (p.y > max.y)
      max.y = p.y;
    else if (p.y < min.y)
      min.y = p.y;
    if (p.z > max.z)
      max.z = p.z;
    else if (p.z < min.z)
      min.z = p.z;
    center.x += polygon_.points[i].x;
    center.y += polygon_.points[i].y;
    center.z += polygon_.points[i].z;
  }
  p.x = polygon_.points[0].x;
  p.y = polygon_.points[0].y;
  p.z = polygon_.points[0].z;
  polygon_mesh_.points.push_back(p);

  center.x /= polygon_.points.size();
  center.y /= polygon_.points.size();
  center.z /= polygon_.points.size();

  scale_.x = max.x - min.x;
  scale_.y = max.y - min.y;
  scale_.z = 0.001;

  if (polygon_.points.size() >= 3 || normal_ != Ogre::Vector3(0, 0, 0))
  {
    object_.pose.position = center;

    mesh_.type = Marker::CUBE;
    mesh_.header.frame_id = frame_id_;
    mesh_.color = color_;
    mesh_.color.a = 0.2;
    mesh_.pose.position = center;
    mesh_.scale = scale_;

    if (normal_ == Ogre::Vector3(0, 0, 0))
    {
      Ogre::Vector3 u = Ogre::Vector3(polygon_.points[1].x - polygon_.points[0].x, polygon_.points[1].y
          - polygon_.points[0].y, polygon_.points[1].z - polygon_.points[0].z);
      Ogre::Vector3 v = Ogre::Vector3(polygon_.points[2].x - polygon_.points[0].x, polygon_.points[2].y
          - polygon_.points[0].y, polygon_.points[2].z - polygon_.points[0].z);
      normal_ = u.crossProduct(v);
    }
    normal_.normalise();

    float nx = normal_.y * 1 - normal_.z * 0;
    float ny = normal_.z * 0 - normal_.x * 1;
    float nz = normal_.x * 0 - normal_.z * 0;

    float length = sqrt(nx * nx + ny * ny + nz * nz);
    nx /= length;
    ny /= length;
    nz /= length;

    float sign = ((nx * 1 + ny * 0 + nz * 0) < 0) ? -1 : 1;
    float angle = acos(normal_.x * 0 + normal_.y * 0 + normal_.z * 1) * sign;

    Eigen::Quaternion<float> q;
    q = Eigen::AngleAxisf(-angle, Eigen::Matrix<float, 3, 1>(nx, ny, nz));
    mesh_.pose.orientation.x = q.x();
    mesh_.pose.orientation.y = q.y();
    mesh_.pose.orientation.z = q.z();
    mesh_.pose.orientation.w = q.w();

    control_.markers.push_back(mesh_);
  }

  control_.always_visible = true;
  control_.interaction_mode = InteractiveMarkerControl::BUTTON;
  control_.markers.push_back(polygon_mesh_);
  object_.controls.push_back(control_);

  createMenu();
}

}
