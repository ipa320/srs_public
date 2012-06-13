/******************************************************************************
 * \file
 *
 * $Id: services_list.h 556 2012-04-11 16:10:40Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 05/03/2012
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
#ifndef BUT_SERVICES_SERVICES_LIST_H_
#define BUT_SERVICES_SERVICES_LIST_H_

#include <srs_ui_but/GetClosestPoint.h>

#include <string>

namespace srs_ui_but
{
	static const std::string PACKAGE_NAME_PREFIX  = "/but_gui";

	/**
	 * Get closest point service topic
	 */
	static const std::string GetClosestPoint_SRV = PACKAGE_NAME_PREFIX + std::string("/get_closest_point");
}

#endif /* BUT_SERVICES_SERVICES_LIST_H_ */

