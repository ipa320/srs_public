/******************************************************************************
 * \file
 *
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 09/08/2012
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
#ifndef BUT_OBJECT_MANAGER_H
#define BUT_OBJECT_MANAGER_H

#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/display.h>
#include <rviz/view_controller.h>
#include "rviz/properties/forwards.h"
#include "rviz/properties/property.h"
#include "rviz/properties/edit_enum_property.h"
#include "rviz/properties/property_manager.h"
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>
#include <cob_object_detection_msgs/Detection.h>
#include <sstream>
#include <string>

#include <srs_ui_but/GetAddedObjects.h>
#include <srs_ui_but/services_list.h>
#include <srs_interaction_primitives/services_list.h>
#include "object_control_pane.h"

namespace srs_ui_but
{

class CButObjectManager : public rviz::Display
{
public:
  /**
   * @brief Constructor
   */
  CButObjectManager(const std::string & name, rviz::VisualizationManager * manager);

  /**
   * @brief Destructor
   */
  ~CButObjectManager();

  virtual void createProperties();

  /**
   * @brief Update display
   */
  virtual void update(float wall_dt, float ros_dt)
  {
  }

protected:
  /*
   * @brief Display enabled callback
   */
  virtual void onEnable();
  /*
   * @brief Display disabledcallback
   */
  virtual void onDisable();
  virtual void targetFrameChanged()
  {
  }
  virtual void fixedFrameChanged()
  {
  }

  /**
   * @brief Gets objects added by the Object Manager
   *
   * @param req  Request of type GetAddedObject.
   * @param res  Response of type GetAddedObject.
   */
  bool getAddedObjects(GetAddedObjects::Request &req, GetAddedObjects::Response &res);

protected:

  // Object controls window
  CObjectControlPane * m_objectWindow_;

private:
  // Service clients and servers
  ros::ServiceClient get_object_client_;
  ros::ServiceServer get_added_objects_service_;

};

} // namespace srs_ui_but

#endif // BUT_OBJECT_MANAGER_H
