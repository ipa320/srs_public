/******************************************************************************
 * \file
 *
 * $Id: UnknownObject.h 397 2012-03-29 12:50:30Z spanel $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 02/12/2011
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

#ifndef UNKNOWNOBJECT_H_
#define UNKNOWNOBJECT_H_

#include "but_gui/Primitive.h"

namespace but_gui
{
/*
 * This class represents an Unknown Object primitive.
 *
 * Unknown Object shows some obstacles in the scene.
 * Unknown Object can be rotated, translated and scaled.
 */

class UnknownObject : public Primitive
{
public:
  /**
   * Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this object
   */
  UnknownObject(InteractiveMarkerServerPtr server, string frame_id, string name);
  /*
   * Inserts Unknown object into Interactive marker server
   */
  void insert();

  // Callbacks
  /**
   * Callback for menu
   */
  void menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback);
  /**
   * Callback for interactive markers
   */
  void uboxCallback(const InteractiveMarkerFeedbackConstPtr &feedback);

private:
  Marker box_;

  void createUnknownBox();
  void createBox();
  void create();
  void createMenu();
};

}

#endif /* UNKNOWNOBJECT_H_ */
