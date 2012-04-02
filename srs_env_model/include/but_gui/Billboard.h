/******************************************************************************
 * \file
 *
 * $Id: Billboard.h 397 2012-03-29 12:50:30Z spanel $
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

#include "Primitive.h"
#include <srs_env_model/BillboardType.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#ifndef BILLBOARD_H_
#define BILLBOARD_H_

using namespace srs_env_model;

namespace but_gui
{
/*
 * This class represents Billboard primitive.
 *
 * Billboard is a simple object created as a plane mesh with texture
 * representing a real world object.
 * Billboard is view facing and can illustrate the movement of the represented object.
 */
class Billboard : public Primitive
{
public:
  /*
   * Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this billboard
   */
  Billboard(InteractiveMarkerServerPtr server, string frame_id, string name);
  /*
   * Sets type to the billboard
   * @param type is billboard's type
   */
  void setType(int type);
  /*
   * Gets billboard's type
   */
  int getType();

  /*
   * Inserts this billboard into Interactive marker server
   */
  void insert();
  /**
   * Callback for menu
   */
  void menuCallback(const InteractiveMarkerFeedbackConstPtr &);

private:
  int billboard_type_;
  Marker mesh_;

  void create();
  void createMenu();
  void createMesh();
};
}
#endif /* BILLBOARD_H_ */
