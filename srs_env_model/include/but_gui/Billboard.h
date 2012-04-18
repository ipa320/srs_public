/******************************************************************************
 * \file
 *
 * $Id: Billboard.h 603 2012-04-16 10:50:03Z xlokaj03 $
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
  Billboard(InteractiveMarkerServerPtr server, string frame_id, string name);

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
