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

#ifndef BUT_CONTEXT_SERVER_H_
#define BUT_CONTEXT_SERVER_H_

#include "but_context/ContextServer.h"
#include <srs_env_model/GetContext.h>
#include <srs_env_model/SetContext.h>
#include <but_context/services_list.h>

namespace but_context
{
ContextServer *contextServer;

/**
 * @brief Sets the current context
 * @param req request of type GetContext
 * @param res response of type GetContext
 * @return true
 */
bool setContext(SetContext::Request &req, SetContext::Response &res)
{
  contextServer->setContext(req.context);
  return true;
}

/**
 * @brief Gets the current context
 * @param req request of type GetContext
 * @param res response of type GetContext
 * @return true
 */
bool getContext(GetContext::Request &req, GetContext::Response &res)
{
  res.context = contextServer->getContext();
  return true;
}

}

#endif /* BUT_CONTEXT_SERVER_H_ */
