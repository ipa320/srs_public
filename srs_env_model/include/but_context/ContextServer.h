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
 * Date: 16.4.2012
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

#ifndef CONTEXTSERVER_H_
#define CONTEXTSERVER_H_

#include <ros/ros.h>
#include <srs_env_model/Context.h>
#include <srs_env_model/ContextChanged.h>
#include <but_context/topics_list.h>

#define BUFFER_SIZE 5

using namespace srs_env_model;

namespace but_context
{
/**
 * This class represents server witch current context and provides method to change
 * and get it. It also publishes topic with the current context, so other nodes
 * are able to adjust their behaviour.
 *
 * @author Tomas Lokaj
 */
class ContextServer
{
public:
  /**
   * @brief Constructor.
   */
  ContextServer();

  /**
   * @brief Destructor.
   */
  virtual ~ContextServer();

  /**
   * @brief Gets the current context
   * @return current context
   */
  Context getContext()
  {
    return context_;
  }

  /**
   * @brief Sets current context
   * @param context is current context
   */
  void setContext(Context context)
  {
    context_ = context;
    publishContext();
  }

  /**
   * @brief Publishes the current context
   */
  void publishContext()
  {
    ContextChanged contextChangedMsg;
    contextChangedMsg.context = context_;
    publisher_.publish(contextChangedMsg);
  }

private:
  Context context_;
  // Node handler
  ros::NodeHandle nh_;
  // Publisher
  ros::Publisher publisher_;
};

}

#endif /* CONTEXTSERVER_H_ */
