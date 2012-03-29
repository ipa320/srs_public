/******************************************************************************
 * \file
 *
 * $Id:$
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

#ifndef OBJECTWITHBOUNDINGBOX_H_
#define OBJECTWITHBOUNDINGBOX_H_

#include <but_gui/BoundingBox.h>

namespace but_gui
{

class ObjectWithBoundingBox : public BoundingBox
{
public:
  /**
   * Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this object
   */
  ObjectWithBoundingBox(InteractiveMarkerServerPtr server, string frame_id, string name);

  /**
   * Creates and inserts object into Interactive Marker Server
   */
  void insert();

  /**
   * Sets object's bounding box
   * @param bounding_box_lwh is bounding box length, width and height
   */
  void setBoundingBoxLWH(Point bounding_box_lwh)
  {
    bounding_box_lwh_ = bounding_box_lwh;
  }

  /*
   * Sets position and orientation
   * @param pose is object's position and orientation
   */
  void setPoseLWH(Pose pose, Point bounding_box_lwh);

  /**
   * Callback
   */
  void objectWithBoundingBoxCallback(const InteractiveMarkerFeedbackConstPtr &feedback);
  /**
   * Menu callback
   */
  void menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback);

private:
  void create();
  void createMenu();
  void createMesh();
  void createBoundingBoxControl();

  Point bounding_box_lwh_;
  bool translated_;
};

}

#endif /* OBJECTWITHBOUNDINGBOX_H_ */

