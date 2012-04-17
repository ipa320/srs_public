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

#ifndef SERVICES_LIST_H_
#define SERVICES_LIST_H_


#define BUT_CONTEXT_PREFIX std::string("/but_context")
#define BUT_CONTEXT_SERVICE_TOPIC(topic) BUT_CONTEXT_PREFIX + std::string(topic)

/*
 * Services for getting and setting the current context
 */
#define BUT_SetContext_SRV BUT_CONTEXT_SERVICE_TOPIC("/set_context")
#define BUT_GetContext_SRV BUT_CONTEXT_SERVICE_TOPIC("/get_context")


#endif /* SERVICES_LIST_H_ */
