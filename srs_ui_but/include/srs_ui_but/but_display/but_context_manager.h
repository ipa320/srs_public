/******************************************************************************
 * \file
 *
 * $Id:
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 07/02/2012
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
#ifndef BUT_CONTEXT_MANAGER_H_
#define BUT_CONTEXT_MANAGER_H_

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <rviz/display.h>
#include <rviz/view_controller.h>
#include "rviz/properties/forwards.h"
#include "rviz/properties/property.h"
#include "rviz/properties/edit_enum_property.h"
#include "rviz/properties/property_manager.h"
#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include "rviz/helpers/color.h"
#include <rviz/render_panel.h>
#include <rviz/window_manager_interface.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <ogre_tools/movable_text.h>
#include <srs_ui_but/but_ogre_tools/text_output.h>

#include <srs_env_model/Context.h>
#include <srs_env_model/ContextChanged.h>
#include <srs_env_model/GetContext.h>
#include <srs_env_model/SetContext.h>

#include <srs_ui_but/topics_list.h>
#include <srs_env_model/services_list.h>
#include <srs_env_model/topics_list.h>

#define ID_MESSAGE_RENDERER "txtMessageRenderer"

namespace srs_ui_but
{
/**
 * @brief This tool displays distances around specified robot's link.
 *
 * @author Tomas Lokaj
 */
class CButContextManager : public rviz::Display

{
public:
  /**
   * @brief Constructor
   */
  CButContextManager(const std::string & name, rviz::VisualizationManager * manager);

  /**
   * @brief Destructor
   */
  virtual ~CButContextManager();

  /**
   * @brief Overriden method from Display
   */
  virtual void targetFrameChanged()
  {
  }
  /**
   * @brief Overriden method from Display
   */
  virtual void fixedFrameChanged()
  {
  }

  /**
   * @brief Creates display properties
   */
  virtual void createProperties();

  /**
   * @brief Updates display
   */
  virtual void update(float wall_dt, float ros_dt);

protected:
  /**
   * @brief Checks if the Context Server is running
   * @return true if the Context Server is running, false otherwise
   */
  bool checkContextServer();

  /**
   * @brief Gets context status
   * @return context status
   */
  int getContextStatus()
  {
    return context_.status_tag;
  }

  /**
   * @brief Sets context status
   * @param status is new context status
   */
  void setContextStatus(int status)
  {
    context_.status_tag = status;
    setContext();
    propertyChanged(m_property_status_);
  }

  /**
   * @brief Gets context action
   * @return context action
   */
  int getContextAction()
  {
    return context_.action_tag;
  }

  /**
   * @brief Sets context action
   * @param status is new context action
   */
  void setContextAction(int action)
  {
    context_.action_tag = action;
    setContext();
    propertyChanged(m_property_action_);
  }

  /**
   * @brief Gets context connection
   * @return context connection
   */
  int getContextConnection()
  {
    return context_.connection_tag;
  }

  /**
   * @brief Sets context connection
   * @param status is new context connection
   */
  void setContextConnection(int connection)
  {
    context_.connection_tag = connection;
    setContext();
    propertyChanged(m_property_connection_);
  }

  /**
   * @brief Gets context collision hazard
   * @return context collision hazard
   */
  int getContextCollisionHazard()
  {
    return context_.collision_hazard_tag;
  }

  /**
   * @brief Sets context collision hazard
   * @param status is new context collision hazard
   */
  void setContextCollisionHazard(int collision_hazard)
  {
    context_.collision_hazard_tag = collision_hazard;
    setContext();
    propertyChanged(m_property_connection_);
  }

  /**
   * @brief Sets new context
   */
  void setContext()
  {
    if (enabled_ && checkContextServer())
    {
      srs_env_model::SetContext context;
      context.request.context = context_;
      set_context_client_.call(context);
    }
  }

  /**
   * @brief Gets new context
   */
  void getContext()
  {
    srs_env_model::GetContext context;
    if (enabled_ && get_context_client_.call(context))
    {
      context_ = context.response.context;
      propertyChanged(m_property_status_);
      propertyChanged(m_property_action_);
      propertyChanged(m_property_connection_);
      propertyChanged(m_property_collision_hazard_);
    }
  }

  /**
   * @brief Callback function for context update
   * @param update
   */
  void contextChangedCallback(const srs_env_model::ContextChangedConstPtr &update)
  {
    context_ = update->context;
    propertyChanged(m_property_status_);
    propertyChanged(m_property_action_);
    propertyChanged(m_property_connection_);
    propertyChanged(m_property_collision_hazard_);
  }

  /**
   * @brief Display is enabled
   */
  virtual void onEnable();

  /**
   * @brief is Display disabled
   */
  virtual void onDisable();

  // Display properties
  rviz::EnumPropertyWPtr m_property_status_, m_property_action_, m_property_connection_, m_property_collision_hazard_;

  // Properties
  srs_env_model::Context context_;

  // Context changed subscriber
  ros::Subscriber sub_;

  // Service clients
  ros::ServiceClient get_context_client_, set_context_client_;

  // Context server properties
  bool context_server_enabled_;

  // Message renderer
  ogre_tools::TextOutput* message_renderer_;
};

} // namespace srs_ui_but

#endif /* BUT_CONTEXT_MANAGER_H_ */

