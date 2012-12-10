/******************************************************************************
 * \file
 *
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 07/23/2012
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
#ifndef INTERACTION_PRIMITIVES_PARAMETERS_LIST_H_
#define INTERACTION_PRIMITIVES_PARAMETERS_LIST_H_

#include <srs_interaction_primitives/services_list.h>

namespace srs_interaction_primitives
{
static const std::string AllowInteraction_PARAM = PACKAGE_NAME_PREFIX + std::string("/allow_object_interaction");
static const std::string ShowPregrasp_PARAM = PACKAGE_NAME_PREFIX + std::string("/show_object_pregrasp");
static const std::string MoveArmToPregraspOnClick_PARAM = PACKAGE_NAME_PREFIX + std::string("/move_arm_to_pregrasp_on_click");

}

#endif /* INTERACTION_PRIMITIVES_PARAMETERS_LIST_H_ */
