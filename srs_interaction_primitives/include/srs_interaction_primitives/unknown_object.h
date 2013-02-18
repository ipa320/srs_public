/******************************************************************************
 * \file
 *
 * $Id: UnknownObject.h 676 2012-04-19 18:32:07Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
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

#pragma once
#ifndef UNKNOWNOBJECT_H_
#define UNKNOWNOBJECT_H_

#include "primitive.h"

namespace srs_interaction_primitives
{
/**
 * This class represents an Unknown Object primitive.
 *
 * Unknown Object will show some obstacles, unreachable objects or dangerous
 * places in the scene, some the operator will be able to avoid collisions
 * or crash of the robot.
 * Unknown Object can be rotated, translated and scaled.
 *
 * @author Tomas Lokaj
 * @see http://ros.org/wiki/srs_env_model#Unknown_Object
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
  UnknownObject(InteractiveMarkerServerPtr server, std::string frame_id, std::string name);

  /**
   * Inserts Unknown object into Interactive marker server
   */
  void insert();

   /**
    * @brief Sets whether the predefined material ("red cross") should be used.
    */
  void useMaterial(bool value) { use_material_ = value; }

  /**
   * Callback for menu
   */
  void menuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
   * Callback for interactive markers
   */
  void uboxCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  
  virtual void setAllowObjectInteraction(bool allow);

private:
  void createUnknownBox();
  void createBox();
  void createColorBox();
  void create();
  void createMenu();

private:
  visualization_msgs::Marker box_, wire_;

  // Menu handlers
  interactive_markers::MenuHandler::EntryHandle menu_handler_interaction_, menu_handler_interaction_movement_,
                                                menu_handler_interaction_rotation_, menu_handler_interaction_scale_;

  // Enables visualization of a predefined model including "red cross"
  bool use_material_;
  
  bool allow_object_interaction_;
  
};

}

#endif /* UNKNOWNOBJECT_H_ */
