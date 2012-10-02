/******************************************************************************
 * \file
 *
 * $Id: Object.h 676 2012-04-19 18:32:07Z xlokaj03 $
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
#ifndef OBJECT_H_
#define OBJECT_H_

#include "bounding_box.h"

namespace srs_interaction_primitives
{

/**
 * @brief Pre-grasp positions ids.
 */
enum
{
  _, GRASP_1, GRASP_2, GRASP_3, GRASP_4, GRASP_5, GRASP_6
};

/**
 * @brief Object resource types.
 */
enum ObjectResource
{
  SHAPE, RESOURCE_FILE
};

/**
 * @brief Pre-grasp position.
 */
struct PreGraspPosition
{
  std::string name;
  bool enabled;
  geometry_msgs::Pose pose;

  PreGraspPosition()
  {
    enabled = false;
  }
};

/**
 * This class represents detected object and it's bounding box.
 *
 * Object with Bounding Box is designed to be the direct output of the detectors.
 * It shows object's mesh together with it's Bounding Box.
 * It can also show pre-grasp positions and can be translated, rotated or scaled.
 *
 * @author Tomas Lokaj
 * @see http://ros.org/wiki/srs_env_model#Object
 */
class Object : public BoundingBox
{
public:
  /**
   * Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this object
   */
  Object(InteractiveMarkerServerPtr server, std::string frame_id, std::string name);

  /**
   * Creates and inserts object into Interactive Marker Server
   */
  void insert();

  /**
   * Sets object's bounding box
   * @param bounding_box_lwh is bounding box length, width and height
   */
  void setBoundingBoxLWH(geometry_msgs::Point bounding_box_lwh)
  {
    bounding_box_lwh_ = bounding_box_lwh;
  }

  /**
   * Gets object's bounding box
   * @return bounding_box_lwh is bounding box length, width and height
   */
  geometry_msgs::Point getBoundingBoxLWH()
  {
    return bounding_box_lwh_;
  }

  /**
   * Sets position orientation of the object and dimension of the bounding box.
   * @param pose is object's position and orientation
   * @param bounding_box_lwh is length, width and height of the bounding box
   */
  void setPoseLWH(geometry_msgs::Pose pose, geometry_msgs::Point bounding_box_lwh);

  /**
   * Allows or disables showing of preg-rasp menu entries
   * @param allow_pregrasp True to show pre-grasp menu entries, False otherwise
   */
  void setAllowPregrasp(bool allow_pregrasp)
  {
    allow_pregrasp_ = allow_pregrasp;
  }

  /**
   * @brief Sets resource file with primitive's model (and material).
   * @param resource path to the resource file
   */
  void setResource(std::string resource)
  {
    object_resource_ = RESOURCE_FILE;
    resource_ = resource;
  }

  /**
   * @brief Gets the resource file with primitive's model (and material).
   * @return resource path to the resource file
   */
  std::string getResource()
  {
    return resource_;
  }

  /**
   * @brief Sets usage of defined material or color
   * @param use_material material (true), color (false)
   */
  void setUseMaterial(bool use_material)
  {
    use_material_ = use_material;
  }

  /**
   * @brief Gets usage of defined material or color
   * @return use_material material (true), color (false)
   */
  bool getUseMaterial()
  {
    return use_material_;
  }

  /**
   * @brief Sets shape of the mesh
   * @param shape is shape
   */
  void setShape(arm_navigation_msgs::Shape shape)
  {
    object_resource_ = SHAPE;
    shape_ = shape;
  }

  /**
   * @brief Gets shape of the mesh
   * @return shape of t he mesh
   */
  arm_navigation_msgs::Shape getShape()
  {
    return shape_;
  }

  /**
   * @brief Allows or denies interaction with Object
   * @param allow is true or false
   */
  void setAllowObjectInteraction(bool allow)
  {
    allow_object_interaction_ = allow;
    if (allow_object_interaction_)
    {
      menu_handler_.setVisible(menu_handler_interaction_, true);
      menu_handler_.setVisible(menu_handler_interaction_movement_, true);
      menu_handler_.setVisible(menu_handler_interaction_rotation_, true);
    }
    else
    {
      menu_handler_.setVisible(menu_handler_interaction_, false);
      menu_handler_.setVisible(menu_handler_interaction_movement_, false);
      menu_handler_.setVisible(menu_handler_interaction_rotation_, false);
      removeMovementControls();
      removeRotationControls();
      server_->insert(object_);
    }

    menu_handler_.reApply(*server_);
    server_->applyChanges();
  }

  /**
   * @brief Sets specified pre-grasp position
   * @param pos_num is specified position number (1-6)
   * @param pose is pre-grasp position and orientation
   */
  void addPreGraspPosition(int pos_id, geometry_msgs::Pose pose);

  /**
   * @brief Removes specified pre-grasp position
   * @param pos_num is specified position number (1-6)
   */
  void removePreGraspPosition(int pos_id);

  /**
   * @brief Adds grasping positions
   */
  void addPregraspPositions();

  /**
   * @brief Removes grasping positions
   */
  void removePregraspPositions();

  /**
   * @brief Updates visible controls
   */
  void updateControls();

  /**
   * Callback
   */
  void objectCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
   * Menu callback
   */
  void menuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

private:
  void create();
  void createMenu();
  void createMesh();
  void createBoundingBoxControl();

  // Object's attributes
  ObjectResource object_resource_;
  visualization_msgs::Marker mesh_;
  arm_navigation_msgs::Shape shape_;
  std::string resource_;
  bool use_material_;
  bool move_arm_to_pregrasp_onclick_;
  bool allow_object_interaction_, allow_pregrasp_;

  geometry_msgs::Point bounding_box_lwh_;
  bool translated_;

  bool show_pregrasp_control_;

  PreGraspPosition pregrasp1_, pregrasp2_, pregrasp3_, pregrasp4_, pregrasp5_, pregrasp6_;

  visualization_msgs::InteractiveMarkerControl pregrasp1Control_, pregrasp2Control_, pregrasp3Control_,
                                               pregrasp4Control_, pregrasp5Control_, pregrasp6Control_;

  // Menu handlers
  interactive_markers::MenuHandler::EntryHandle menu_handler_interaction_, menu_handler_interaction_movement_,
                                                menu_handler_interaction_rotation_, menu_handler_show_pregrasp_,
                                                menu_handler_move_to_pregrasp_;
};

}

#endif /* OBJECT_H_ */

